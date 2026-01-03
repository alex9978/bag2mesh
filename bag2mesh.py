import argparse
import logging
import sys
import time
import struct
from pathlib import Path
import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader
from tqdm import tqdm
import cv2
import copy

# ANSI Colors
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"

class ColoredFormatter(logging.Formatter):
    def format(self, record):
        log_fmt = "%(levelname)s - %(message)s"
        if record.levelno == logging.WARNING:
            log_fmt = f"{YELLOW}%(levelname)s - %(message)s{RESET}"
        elif record.levelno == logging.ERROR:
            log_fmt = f"{RED}%(levelname)s - %(message)s{RESET}"
        elif record.levelno == logging.INFO:
            log_fmt = f"{GREEN}%(levelname)s{RESET} - %(message)s"
        
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)

# Configure logging
root_logger = logging.getLogger()
root_logger.setLevel(logging.INFO)

# Suppress Open3D warnings
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

# Clear existing handlers if any (to avoid duplication)
if root_logger.hasHandlers():
    root_logger.handlers.clear()

# Console handler (no time, colored)
c_handler = logging.StreamHandler(sys.stdout)
c_handler.setFormatter(ColoredFormatter())
root_logger.addHandler(c_handler)

# File handler (with time, no color)
f_handler = logging.FileHandler("bag2mesh.log", mode='w')
f_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
root_logger.addHandler(f_handler)

logger = logging.getLogger(__name__)

class SimpleImage:
    def __init__(self, width, height, encoding, data):
        self.width = width
        self.height = height
        self.encoding = encoding
        self.data = data

def manual_decode_image(rawdata):
    """
    Manually decodes a ROS1 sensor_msgs/Image message.
    """
    try:
        pos = 0
        # seq (4) + time (8)
        pos += 12
        
        # frame_id length
        if pos + 4 > len(rawdata): return None
        frame_id_len = struct.unpack('<I', rawdata[pos:pos+4])[0]
        pos += 4 + frame_id_len
        
        # height
        if pos + 4 > len(rawdata): return None
        height = struct.unpack('<I', rawdata[pos:pos+4])[0]
        pos += 4
        # width
        if pos + 4 > len(rawdata): return None
        width = struct.unpack('<I', rawdata[pos:pos+4])[0]
        pos += 4
        
        # encoding
        if pos + 4 > len(rawdata): return None
        encoding_len = struct.unpack('<I', rawdata[pos:pos+4])[0]
        pos += 4
        encoding = rawdata[pos:pos+encoding_len].decode('utf-8')
        pos += encoding_len
        
        # is_bigendian
        pos += 1
        
        # step
        pos += 4
        
        # data length
        if pos + 4 > len(rawdata): return None
        data_len = struct.unpack('<I', rawdata[pos:pos+4])[0]
        pos += 4
        
        data = rawdata[pos:pos+data_len]
        
        return SimpleImage(width, height, encoding, data)
    except Exception as e:
        logger.debug(f"Manual decode failed: {e}")
        return None

def extract_points_from_bag(bag_path, output_mesh, voxel_size=0.02, topic_name=None, depth_scale_override=None, step=2, 
                            poisson_depth=11, density_threshold=0.1, 
                            colored_icp_min_fitness=0.5, colored_icp_search_ratio=2.0,
                            window_size=20, coarse_threshold_multiplier=15, fine_threshold_multiplier=3,
                            keyframe_movement_threshold=0.05, keyframe_rotation_threshold=0.17,
                            outlier_nb_neighbors=20, outlier_std_ratio=2.0,
                            max_translation_jump=0.3, max_rotation_jump=0.5,
                            linear_fit=True, remove_outliers=True, save_cloud_path=None):
    points_list = []
    colors_list = []
    normals_list = []
    
    logger.info(f"Opening bag file: {bag_path}")
    
    try:
        with AnyReader([Path(bag_path)]) as reader:
            # --- Inspect Bag Integration ---
            logger.info(f"Inspecting bag: {len(reader.connections)} topics found.")
            for connection in reader.connections:
                logger.info(f" {CYAN}{connection.topic}{RESET} [{MAGENTA}{connection.msgtype}{RESET}]")
            logger.info("-" * 80)
            # -------------------------------

            # 1. Check for PointCloud2
            pc_connections = [x for x in reader.connections if x.msgtype == 'sensor_msgs/msg/PointCloud2']
            
            if pc_connections and (not topic_name or topic_name in [c.topic for c in pc_connections]):
                logger.info(f"Found PointCloud2 topics: {[c.topic for c in pc_connections]}")
                # PC2 logic omitted for brevity
            
            if not pc_connections:
                logger.info("No PointCloud2 found. Searching for Depth and Color images...")
                
                # Prioritize aligned topics
                depth_topic = None
                color_topic = None
                info_topic = None
                
                all_topics = [c.topic for c in reader.connections]
                
                # Search for aligned depth first
                for t in all_topics:
                    if 'aligned_depth_to_color' in t and 'image' in t:
                        depth_topic = t
                        break
                if not depth_topic:
                    for t in all_topics:
                        if 'Depth' in t and 'image' in t:
                            depth_topic = t
                            break
                            
                # Search for color
                for t in all_topics:
                    if 'Color' in t and 'image' in t:
                        color_topic = t
                        break

                # Search for CameraInfo
                for t in all_topics:
                    if 'camera_info' in t.lower():
                        # Try to match with depth topic if possible
                        if depth_topic:
                            # Check if they share the same prefix (device/sensor/stream)
                            # depth_topic: /device_0/sensor_0/Depth_0/image/data
                            # info_topic: /device_0/sensor_0/Depth_0/info/camera_info
                            depth_parts = depth_topic.split('/')
                            t_parts = t.split('/')
                            
                            if len(depth_parts) >= 4 and len(t_parts) >= 4:
                                # Compare device, sensor, and stream name
                                if depth_parts[1:4] == t_parts[1:4]:
                                    info_topic = t
                                    break
                        
                        # Fallback to any camera_info if we haven't found a match yet
                        if not info_topic:
                            info_topic = t
                        
                if not depth_topic:
                    logger.error("Could not find Depth topic.")
                    return None
                    
                logger.info(f"Using Depth: {depth_topic}")
                logger.info(f"Using Color: {color_topic}")
                
                # Search for Depth Units
                depth_scale = 1000.0 # Default 1mm
                depth_units_topic = None
                
                if depth_scale_override:
                    depth_scale = depth_scale_override
                    logger.info(f"Using forced depth scale: {depth_scale}")
                else:
                    for t in all_topics:
                        if 'Depth_Units/value' in t:
                            depth_units_topic = t
                            break
                    
                    if depth_units_topic:
                         logger.info(f"Found Depth Units topic: {depth_units_topic}")
                         # Read one message to get the value
                         du_conn = [c for c in reader.connections if c.topic == depth_units_topic]
                         for connection, timestamp, rawdata in reader.messages(connections=du_conn):
                             try:
                                 msg = reader.deserialize(rawdata, connection.msgtype)
                                 # std_msgs/Float32 has a 'data' field
                                 d_unit = getattr(msg, 'data', None)
                                 if d_unit is not None and d_unit > 0:
                                     depth_scale = 1.0 / d_unit
                                     logger.info(f"Read Depth Units: {d_unit} -> Setting depth_scale to {depth_scale}")
                                     break
                             except Exception as e:
                                 logger.warning(f"Failed to read Depth Units: {e}")
                                 pass

                connections = [c for c in reader.connections if c.topic in [depth_topic, color_topic, info_topic]]
                
                intrinsics = None
                
                # First pass: get intrinsics
                if info_topic:
                    for connection, timestamp, rawdata in reader.messages(connections=[c for c in connections if c.topic == info_topic]):
                        try:
                            msg = reader.deserialize(rawdata, connection.msgtype)
                            intrinsics = o3d.camera.PinholeCameraIntrinsic()
                            K_matrix = getattr(msg, 'k', getattr(msg, 'K', None))
                            if K_matrix is not None:
                                intrinsics.set_intrinsics(msg.width, msg.height, K_matrix[0], K_matrix[4], K_matrix[2], K_matrix[5])
                                logger.info("Found Camera Intrinsics.")
                                break
                        except Exception as e:
                            pass
                
                if not intrinsics:
                    logger.warning("No CameraInfo found. Using default intrinsics.")
                    intrinsics = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

                logger.info("Processing Image messages...")
                
                last_depth = None
                last_color = None
                frame_count = 0
                processed_frames = 0
                
                # Registration variables
                prev_pcd_down = None
                global_transform = np.identity(4)
                last_relative_transform = np.identity(4) # For constant velocity model
                last_saved_pose = np.identity(4) # For keyframe selection
                
                # Count total messages for progress bar
                total_msgs = sum(1 for _ in reader.messages(connections=connections))
                
                current_msg = 0
                pbar = tqdm(total=total_msgs, desc="Reading Images")
                
                skipped_frames_count = 0
                
                for connection, timestamp, rawdata in reader.messages(connections=connections):
                    current_msg += 1
                    pbar.update(1)

                    try:
                        msg = reader.deserialize(rawdata, connection.msgtype)
                    except Exception as e:
                        if 'Image' in connection.msgtype:
                            msg = manual_decode_image(rawdata)
                            if not msg: continue
                        else:
                            continue
                    
                    encoding = getattr(msg, 'encoding', None)
                    if not encoding: continue

                    if connection.topic == depth_topic:
                        try:
                            if '16UC1' in encoding or 'mono16' in encoding or 'z16' in encoding:
                                img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                                last_depth = o3d.geometry.Image(img)
                        except Exception as e:
                            logger.error(f"Error processing depth: {e}")

                    elif connection.topic == color_topic:
                        try:
                            if encoding == 'rgb8':
                                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                                last_color = o3d.geometry.Image(img)
                            elif encoding == 'bgr8':
                                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                                last_color = o3d.geometry.Image(img)
                        except Exception as e:
                            logger.error(f"Error processing color: {e}")
                    
                    if last_depth and last_color:
                        frame_count += 1
                        if frame_count % step == 0:  # Process every Nth frame
                            try:
                                depth_np = np.asarray(last_depth)
                                color_np = np.asarray(last_color)
                                d_h, d_w = depth_np.shape
                                c_h, c_w, _ = color_np.shape
                                
                                if (d_w != c_w) or (d_h != c_h):
                                    color_resized = cv2.resize(color_np, (d_w, d_h), interpolation=cv2.INTER_LINEAR)
                                    last_color = o3d.geometry.Image(color_resized)
                                
                                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                                    last_color, last_depth, 
                                    depth_scale=depth_scale, depth_trunc=3.0, convert_rgb_to_intensity=False
                                )
                                pcd_frame = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
                                
                                # Basic orientation fix (Camera to World convention)
                                # Removed to avoid confusion. Working in Camera Optical Frame (Z-forward, Y-down).
                                # pcd_frame.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                                
                                # --- ICP Registration ---
                                # Downsample for registration speed
                                pcd_down = pcd_frame.voxel_down_sample(voxel_size=voxel_size)
                                
                                # Clean noise (flying pixels) per frame to improve registration
                                pcd_down, _ = pcd_down.remove_statistical_outlier(nb_neighbors=outlier_nb_neighbors, std_ratio=outlier_std_ratio)
                                
                                if len(pcd_down.points) < 100:
                                    logger.debug("Frame has too few points after downsampling. Skipping registration.")
                                    continue

                                # Estimate normals with a larger radius to be more robust
                                pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 4, max_nn=50))
                                pcd_down.orient_normals_towards_camera_location(np.array([0., 0., 0.]))
                                
                                is_bad_registration = False

                                if points_list:
                                    # Align current frame to LOCAL MAP (Last N keyframes in Global Space)
                                    try:
                                        # Build Local Map from last N keyframes
                                        target_points = np.concatenate(points_list[-window_size:], axis=0)
                                        target_normals = np.concatenate(normals_list[-window_size:], axis=0)
                                        
                                        target_pcd = o3d.geometry.PointCloud()
                                        target_pcd.points = o3d.utility.Vector3dVector(target_points)
                                        target_pcd.normals = o3d.utility.Vector3dVector(target_normals)
                                        
                                        # Coarse-to-Fine ICP Strategy
                                        coarse_threshold = voxel_size * coarse_threshold_multiplier
                                        fine_threshold = voxel_size * fine_threshold_multiplier
                                        
                                        # Initial Guess: Predicted Global Pose
                                        # Prediction = Current Global * Velocity
                                        predicted_global = global_transform @ last_relative_transform
                                        
                                        # 1. Coarse alignment (Local -> Global)
                                        reg_coarse = o3d.pipelines.registration.registration_icp(
                                            pcd_down, target_pcd, coarse_threshold, predicted_global,
                                            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
                                        )
                                        
                                        # 2. Fine alignment
                                        reg_fine = o3d.pipelines.registration.registration_icp(
                                            pcd_down, target_pcd, fine_threshold, reg_coarse.transformation,
                                            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
                                        )
                                        
                                        # 3. Colored ICP (only if geometric is decent but not perfect)
                                        # Increased threshold to 0.5 to prevent "No correspondences" error in Open3D
                                        if reg_fine.fitness > colored_icp_min_fitness and reg_fine.fitness < 0.9: 
                                            try:
                                                # Use a slightly larger threshold for colored registration to find correspondences
                                                # Need colors for target
                                                target_colors = np.concatenate(colors_list[-window_size:], axis=0)
                                                target_pcd.colors = o3d.utility.Vector3dVector(target_colors)
                                                
                                                colored_threshold = fine_threshold * colored_icp_search_ratio
                                                reg_colored = o3d.pipelines.registration.registration_colored_icp(
                                                    pcd_down, target_pcd, colored_threshold, reg_fine.transformation,
                                                    o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                                                    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=20)
                                                )
                                                if reg_colored.fitness > reg_fine.fitness:
                                                    reg_fine = reg_colored
                                            except Exception:
                                                pass

                                        # Critical Failure Check
                                        # Calculate translation magnitude (relative to previous global)
                                        # reg_fine.transformation is the NEW Global Pose
                                        # We need to compare it to the OLD Global Pose to check for jumps
                                        
                                        new_global_transform = reg_fine.transformation
                                        relative_motion = np.linalg.inv(global_transform) @ new_global_transform
                                        trans_mag = np.linalg.norm(relative_motion[:3, 3])
                                        
                                        # Rotation magnitude check
                                        trace = np.trace(relative_motion[:3, :3])
                                        trace = np.clip(trace, -1, 3)
                                        rot_mag = np.arccos((trace - 1) / 2)
                                        
                                        is_bad_registration = False
                                        
                                        # Stricter thresholds to eliminate last ghost
                                        if reg_fine.fitness < 0.1: # Increased from 0.06
                                            is_bad_registration = True
                                        elif reg_fine.inlier_rmse > voxel_size * 3.0: # Stricter RMSE (was 5.0)
                                            is_bad_registration = True
                                        elif trans_mag > max_translation_jump: # Stricter jump limit
                                            is_bad_registration = True
                                        elif rot_mag > max_rotation_jump: # Rotation limit
                                            is_bad_registration = True
                                            
                                        if is_bad_registration:
                                            # Instead of freezing (continue), we use Constant Velocity Model
                                            # This keeps the camera moving, giving a chance to recover tracking later.
                                            # Update global using velocity
                                            global_transform = global_transform @ last_relative_transform
                                            skipped_frames_count += 1
                                        else:
                                            # Update global using registration result
                                            global_transform = new_global_transform
                                            # Update velocity (New Global * Old Global Inverse)
                                            last_relative_transform = relative_motion
                                        
                                    except Exception as e:
                                        logger.warning(f"ICP Registration failed: {e}")
                                        # Fallback to constant velocity on exception
                                        global_transform = global_transform @ last_relative_transform
                                        is_bad_registration = True
                                
                                # If registration was bad, do NOT add points to the map to avoid ghosting/artifacts.
                                # But we DID update the transform (prediction), so tracking continues.
                                if is_bad_registration:
                                    continue

                                # --- Accumulate Downsampled Frame with Normals ---
                                # Transform to global space
                                # Note: pcd_down is still in LOCAL coordinates here.
                                # We need to transform it using the NEW global_transform
                                pcd_down_global = copy.deepcopy(pcd_down)
                                pcd_down_global.transform(global_transform)
                                
                                # Keyframe Selection: Only save frame if camera moved enough
                                # This prevents accumulating too many overlapping points (ghosting/thickness)
                                # Calculate motion relative to last saved pose
                                relative_to_last = np.linalg.inv(last_saved_pose) @ global_transform
                                dist_moved = np.linalg.norm(relative_to_last[:3, 3])
                                
                                # Rotation magnitude (trace method)
                                trace = np.trace(relative_to_last[:3, :3])
                                trace = np.clip(trace, -1, 3)
                                angle_moved = np.arccos((trace - 1) / 2)
                                
                                # Thresholds: Increased to 10cm or 10 degrees to reduce overlap
                                # Always save the first frame (points_list is empty)
                                if not points_list or dist_moved > keyframe_movement_threshold or angle_moved > keyframe_rotation_threshold:
                                    points_list.append(np.asarray(pcd_down_global.points))
                                    colors_list.append(np.asarray(pcd_down_global.colors))
                                    normals_list.append(np.asarray(pcd_down_global.normals))
                                    last_saved_pose = global_transform
                                # ------------------------
                                
                                processed_frames += 1
                                last_depth = None
                                last_color = None
                            except Exception as e:
                                logger.error(f"Error processing frame: {e}")
                
                pbar.close()

        if skipped_frames_count > 0:
            logger.warning(f"Skipped {skipped_frames_count} frames due to low ICP fitness (tracking lost).")

        if not points_list:
            return None

        logger.info(f"Accumulated {processed_frames} frames.")
        all_points = np.concatenate(points_list, axis=0)
        all_colors = np.concatenate(colors_list, axis=0)
        all_normals = np.concatenate(normals_list, axis=0)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        pcd.colors = o3d.utility.Vector3dVector(all_colors)
        pcd.normals = o3d.utility.Vector3dVector(all_normals)
        
        # Save raw point cloud for debugging
        if save_cloud_path:
            logger.info(f"Saving raw point cloud to {save_cloud_path}...")
            o3d.io.write_point_cloud(save_cloud_path, pcd)
        
        # Call process_and_mesh directly here since we have the pcd
        process_and_mesh(pcd, output_mesh, voxel_size, poisson_depth, density_threshold, outlier_nb_neighbors, outlier_std_ratio, linear_fit, remove_outliers)
        
        return pcd

    except Exception as e:
        logger.error(f"Extraction Error: {e}")
        return None

def process_and_mesh(pcd, output_path, voxel_size=0.02, poisson_depth=11, density_threshold=0.1, outlier_nb_neighbors=20, outlier_std_ratio=2.0, linear_fit=True, remove_outliers=True):
    logger.info(f"Total points extracted: {len(pcd.points)}")
    
    if len(pcd.points) == 0:
        logger.warning("Point cloud is empty.")
        return

    # 1. Refine Point Cloud (No Downsampling)
    # We skip downsampling here to preserve the maximum detail captured during tracking.
    # The cloud is already voxelized at 'voxel_size' from the extraction loop.
    
    # 1.5 Outlier Removal (Clean up noise before meshing)
    if remove_outliers:
        logger.info("Removing statistical outliers...")
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=outlier_nb_neighbors, std_ratio=outlier_std_ratio)
        logger.info(f"Points for meshing: {len(pcd.points)}")

    # 2. Normals
    # We trust the normals from the tracking loop (oriented towards camera).
    # Re-estimating them globally risks flipping orientation.
    if not pcd.has_normals():
        logger.info("Estimating normals (none present)...")
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(100)

    # 3. Surface Reconstruction
    logger.info(f"Running Poisson Surface Reconstruction (Depth={poisson_depth})...")
    try:
        # Depth 12 = Very high resolution (octree depth)
        # linear_fit=True = Better alignment with points
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=poisson_depth, linear_fit=linear_fit)
    except Exception as e:
        logger.error(f"Poisson failed: {e}")
        return

    # 4. Clean up
    logger.info(f"Cleaning mesh artifacts (Threshold={density_threshold})...")
    # Aggressive trimming: Remove 40% of vertices with low density.
    # This removes the "bubble" effect and parts where data was sparse (filled holes).
    vertices_to_remove = densities < np.quantile(densities, density_threshold)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    # 5. Colorize
    if pcd.has_colors():
        logger.info("Transferring colors to mesh...")
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        mesh_vertices = np.asarray(mesh.vertices)
        mesh_colors = []
        colors_arr = np.asarray(pcd.colors)
        
        num_vertices = len(mesh_vertices)
        
        for i in tqdm(range(num_vertices), desc="Colorizing"):
            [k, idx, _] = pcd_tree.search_knn_vector_3d(mesh_vertices[i], 1)
            mesh_colors.append(colors_arr[idx[0]])
            
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.array(mesh_colors))

    # 6. Save
    logger.info(f"Saving mesh to {output_path}...")
    o3d.io.write_triangle_mesh(output_path, mesh)
    
    logger.info("Done!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS Bag 3D scan to Mesh")
    parser.add_argument("input_bag", help="Path to input .bag file")
    parser.add_argument("output_mesh", help="Path to output mesh file (e.g., output.ply or output.obj)")
    parser.add_argument("--topic", help="Specific PointCloud2 topic to use", default=None)
    parser.add_argument("--voxel_size", type=float, default=0.02, 
                        help="Voxel size for downsampling (default: 0.02).")
    parser.add_argument("--depth_scale", type=float, default=None, help="Force depth scale (e.g. 1000.0)")
    parser.add_argument("--step", type=int, default=2, help="Process every Nth frame (default: 2)")
    parser.add_argument("--poisson_depth", type=int, default=11, help="Poisson reconstruction depth (default: 11). Lower for smoother mesh.")
    parser.add_argument("--density_threshold", type=float, default=0.1, help="Trim threshold (default: 0.1). Lower for fuller mesh, Higher for cleaner mesh.")
    parser.add_argument("--colored_icp_min_fitness", type=float, default=0.5, help="Min fitness to run Colored ICP (default: 0.5)")
    parser.add_argument("--colored_icp_search_ratio", type=float, default=2.0, help="Search radius multiplier for Colored ICP (default: 2.0)")
    
    # Advanced Tracking Parameters
    parser.add_argument("--window_size", type=int, default=20, help="Local map window size (default: 20)")
    parser.add_argument("--coarse_threshold_multiplier", type=float, default=15.0, help="Coarse ICP threshold multiplier (default: 15.0)")
    parser.add_argument("--fine_threshold_multiplier", type=float, default=3.0, help="Fine ICP threshold multiplier (default: 3.0)")
    
    # Keyframe Parameters
    parser.add_argument("--keyframe_movement_threshold", type=float, default=0.05, help="Min movement to save keyframe (default: 0.05m)")
    parser.add_argument("--keyframe_rotation_threshold", type=float, default=0.17, help="Min rotation to save keyframe (default: 0.17rad)")
    
    # Outlier Removal Parameters
    parser.add_argument("--outlier_nb_neighbors", type=int, default=20, help="Outlier removal neighbors (default: 20)")
    parser.add_argument("--outlier_std_ratio", type=float, default=2.0, help="Outlier removal std ratio (default: 2.0)")
    parser.add_argument("--skip_outlier_removal", action="store_true", help="Skip statistical outlier removal before meshing")
    
    # Validation Parameters
    parser.add_argument("--max_translation_jump", type=float, default=0.3, help="Max allowed translation jump (default: 0.3m)")
    parser.add_argument("--max_rotation_jump", type=float, default=0.5, help="Max allowed rotation jump (default: 0.5rad)")

    # Meshing Parameters
    parser.add_argument("--no_linear_fit", action="store_true", help="Disable linear fit in Poisson reconstruction (use if mesh is too noisy)")
    
    # Output Parameters
    parser.add_argument("--save_cloud", help="Path to save the raw point cloud (e.g., cloud.ply). If not provided, cloud is not saved.", default=None)

    args = parser.parse_args()
    
    # extract_points_from_bag now handles the full pipeline
    extract_points_from_bag(args.input_bag, args.output_mesh, args.voxel_size, args.topic, args.depth_scale, args.step, 
                            args.poisson_depth, args.density_threshold, 
                            args.colored_icp_min_fitness, args.colored_icp_search_ratio,
                            args.window_size, args.coarse_threshold_multiplier, args.fine_threshold_multiplier,
                            args.keyframe_movement_threshold, args.keyframe_rotation_threshold,
                            args.outlier_nb_neighbors, args.outlier_std_ratio,
                            args.max_translation_jump, args.max_rotation_jump,
                            linear_fit=not args.no_linear_fit, remove_outliers=not args.skip_outlier_removal,
                            save_cloud_path=args.save_cloud)

# bag2mesh

**bag2mesh** is a powerful, standalone Python tool designed to convert **ROS Bag files** (specifically from RGB-D cameras like Intel RealSense) into high-quality **3D Meshes (.ply)**.

Unlike many other tools, **this does not require a ROS installation**. It uses `rosbags` to read data directly and `Open3D` for advanced processing, making it lightweight and easy to run on Windows, Linux, or macOS.

![Logo](logo.png)

## ✨ Key Features

*   **No ROS Required**: Runs purely on Python.
*   **Robust Tracking**: Implements a **Local Map ICP** strategy with Constant Velocity prediction to maintain tracking even during fast movements.
*   **Anti-Ghosting**: Smart keyframe selection and strict validation logic prevent "double vision" or ghosting artifacts in the final model.
*   **High-Fidelity Meshing**: Uses **Poisson Surface Reconstruction** with customizable depth and density trimming to produce watertight or clean surfaces.
*   **Auto-Colorization**: Projects color data from the RGB camera onto the generated mesh.
*   **Fully Parametric**: Almost every aspect of the pipeline (tracking, filtering, meshing) can be tuned via command-line arguments.

## 🚀 Installation

1.  Clone the repository:
    ```bash
    git clone https://github.com/yourusername/bag2mesh.git
    cd bag2mesh
    ```

2.  Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```
    *(Dependencies: `open3d`, `rosbags`, `numpy`, `opencv-python`, `tqdm`)*

## 💻 Usage

### Basic Example
Convert a bag file to a mesh using default settings (good for most room-scale scans):

```bash
python bag2mesh.py input.bag output.ply
```

### High Quality Object Scan
For scanning a small object (e.g., a chair) with high detail:

```bash
python bag2mesh.py object.bag chair.ply --voxel_size 0.01 --poisson_depth 11 --density_threshold 0.2
```

### Fast Preview
To quickly check the geometry without waiting for high-res processing:

```bash
python bag2mesh.py large_scan.bag preview.ply --step 5 --voxel_size 0.05 --poisson_depth 8
```

---

## ⚙️ Parameter Guide

This tool is highly configurable. Below is a detailed explanation of all parameters, their defaults, and how to tune them.

### 1. General Settings

| Parameter | Default | Description | Tuning Guide |
| :--- | :--- | :--- | :--- |
| `--voxel_size` | `0.02` | The resolution of the point cloud (in meters). | **Decrease (e.g., 0.01)** for small objects/high detail. **Increase (e.g., 0.05)** for large rooms or faster processing. Warning: Too small makes tracking harder. |
| `--step` | `2` | Process every N-th frame. | **Increase (e.g., 5 or 10)** to speed up processing or if the camera moved very slowly. **Set to 1** for maximum data density. |
| `--depth_scale` | `None` | Force a specific depth scale factor. | Use `1000.0` if your depth is in millimeters but the bag metadata is missing or wrong. |
| `--topic` | `None` | Specific PointCloud2 topic to use. | Use this if the bag contains multiple point clouds and you want to select a specific one. |

### 2. Meshing Quality (Poisson)

| Parameter | Default | Description | Tuning Guide |
| :--- | :--- | :--- | :--- |
| `--poisson_depth` | `11` | Depth of the Octree for reconstruction. Controls mesh resolution. | **Increase (12-13)** for extremely sharp details (requires more RAM). **Decrease (8-9)** for smoother, blob-like shapes or fast previews. |
| `--density_threshold` | `0.1` | Trims vertices with low point support. | **Increase (0.2 - 0.5)** to remove "bubbles", filled holes, and artifacts (makes mesh "thinner"). **Decrease (0.0)** to fill holes and keep more geometry. |
| `--no_linear_fit` | `False` | Disable linear interpolation in Poisson. | Use this flag if the mesh looks too noisy or "spiky". |

### 3. Tracking & Stability (ICP)

| Parameter | Default | Description | Tuning Guide |
| :--- | :--- | :--- | :--- |
| `--window_size` | `20` | Number of previous keyframes to align against (Local Map). | **Increase** for more stable tracking in feature-poor areas. **Decrease** for faster processing. |
| `--coarse_threshold_multiplier` | `15.0` | Search radius for initial alignment (`voxel_size * multiplier`). | **Increase** if the camera moves very fast between frames. |
| `--fine_threshold_multiplier` | `3.0` | Search radius for fine alignment. | **Decrease** for tighter alignment precision (requires good initial guess). |
| `--colored_icp_min_fitness` | `0.5` | Min geometric fitness required to attempt Color ICP. | **Increase** if you see "No correspondences" errors or bad color alignment. |
| `--colored_icp_search_ratio` | `2.0` | Search radius multiplier for Color ICP. | **Increase** to allow color alignment to search further away. |

### 4. Keyframe Selection (Anti-Ghosting)

| Parameter | Default | Description | Tuning Guide |
| :--- | :--- | :--- | :--- |
| `--keyframe_movement_threshold` | `0.05` | Min movement (meters) to save a frame. | **Increase (e.g., 0.10 - 0.20)** to reduce point overlap and ghosting. **Decrease** to create a denser cloud. |
| `--keyframe_rotation_threshold` | `0.17` | Min rotation (radians) to save a frame. | **Increase** to save fewer frames during rotation. |

### 5. Outlier Removal & Validation

| Parameter | Default | Description | Tuning Guide |
| :--- | :--- | :--- | :--- |
| `--outlier_nb_neighbors` | `20` | Neighbors to check for noise filtering. | **Increase** for aggressive noise removal. |
| `--outlier_std_ratio` | `2.0` | Standard deviation ratio for noise. | **Decrease (e.g., 1.0)** to remove more points (cleaner but might lose details). |
| `--skip_outlier_removal` | `False` | Skip the final outlier removal step. | Use this flag if you want to keep all points, even noisy ones. |
| `--max_translation_jump` | `0.3` | Max allowed movement (m) per frame. | **Increase** if you move the camera very fast. **Decrease** if tracking is getting lost/jumping. |
| `--max_rotation_jump` | `0.5` | Max allowed rotation (rad) per frame. | **Increase** if you rotate the camera very fast. **Decrease** to prevent tracking errors during rotation. |

### 6. Output Options

| Parameter | Default | Description | Tuning Guide |
| :--- | :--- | :--- | :--- |
| `--save_cloud` | `None` | Path to save the raw point cloud (e.g., `cloud.ply`). | Use this if you want to inspect the raw point cloud before meshing. |

## 🛠 Troubleshooting

*   **"Ghosting" (Double objects):**
    *   Increase `--keyframe_movement_threshold` (e.g., to `0.15` or `0.20`).
    *   Increase `--voxel_size` slightly.
    *   Ensure `--max_translation_jump` is not too high.

*   **Mesh is too "bubbly" or fills empty spaces:**
    *   Increase `--density_threshold` (e.g., to `0.4`).

*   **Mesh is too rough/noisy:**
    *   Decrease `--poisson_depth` (e.g., to `9` or `10`).
    *   Decrease `--outlier_std_ratio` (e.g., to `1.0`).

*   **Tracking lost (Mesh is scrambled):**
    *   Increase `--coarse_threshold_multiplier`.
    *   Increase `--window_size`.
    *   Try processing every frame (`--step 1`).

## 📄 License

MIT License

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

### Rotating Object Scan (Turntable)
If scanning an object on a turntable with a fixed camera, use `--max_depth` to remove the background:

```bash
python bag2mesh.py turntable.bag object.ply --max_depth 0.8
```

### Fast Preview
To quickly check the geometry without waiting for high-res processing:

```bash
python bag2mesh.py large_scan.bag preview.ply --step 5 --voxel_size 0.05 --poisson_depth 8
```

---

## ⚙️ Parameter Guide

For a detailed explanation of all parameters, defaults, and tuning guides, please refer to the **[Wiki Documentation](bag2mesh.wiki/Home.md#usage--parameters)**.

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

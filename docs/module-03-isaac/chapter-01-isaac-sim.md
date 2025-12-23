---
sidebar_position: 2
title: '3.1: Isaac Sim & Synthetic Data'
---

# Chapter 3.1: Isaac Sim & Synthetic Data

NVIDIA Isaac Sim is a powerful, GPU-accelerated robotics simulation platform built on Omniverse. It provides a highly realistic, physically accurate environment for developing, testing, and training AI-powered robots. A key feature of Isaac Sim is its ability to generate vast amounts of **synthetic data**, which is invaluable for machine learning tasks where real-world data collection can be expensive, time-consuming, or dangerous.

## 1. What is NVIDIA Isaac Sim?

Isaac Sim is more than just a simulator; it's a comprehensive development environment that allows you to:

-   **Simulate Robotics**: Realistic physics, sensor models (LiDAR, cameras, IMUs), and robot control interfaces (ROS 2).
-   **Synthetic Data Generation**: Programmatically generate high-quality, labeled data for perception model training. This includes RGB, depth, semantic segmentation, bounding boxes, and more.
-   **World Building**: Create complex 3D environments using USD (Universal Scene Description).
-   **Hardware Integration**: Leverage NVIDIA GPUs for accelerated simulation and AI workloads.

## 2. Photorealistic Simulation vs. Physics-Only Simulation

Unlike physics-only simulators (like Gazebo, which focuses primarily on physics interactions), Isaac Sim offers **photorealistic rendering**. This means it can simulate light, shadows, reflections, and materials with high fidelity, making the generated data appear indistinguishable from real-world camera captures. This is crucial for training computer vision models that need to perform well in diverse lighting and environmental conditions.

## 3. USD (Universal Scene Description) Basics

**USD** is a high-performance, extensible software platform for collaboratively constructing animated 3D scenes. It's the native format for Omniverse and, by extension, Isaac Sim. You interact with USD files to define your robots, environments, and simulation setups.

-   **Structure**: USD files (`.usd`, `.usdc`, `.usda`) can reference other USD files, allowing for modular and layered scene composition.
-   **Assets**: Models, materials, lights, cameras, and even entire environments are represented as prims (primitives) within a USD stage.

## 4. Creating Humanoid Scenes

In Isaac Sim, you can load existing humanoid models or import your own (e.g., from URDF). Once loaded, you can manipulate their poses, add sensors, and define interactions programmatically using Python scripting.

Here's a simple example of how to load a humanoid model (assuming it's available in Isaac Sim's asset library) into a new scene. This Python script is typically run within the Isaac Sim environment.

**File: `code-examples/module3-isaac-ai-brain/chapter3.1/isaac_sim_setup/simple_humanoid_scene.usd`**
*(Note: For simplicity, this is an empty USD placeholder. Actual scene creation is done programmatically or in the Isaac Sim UI, but this file serves as the target for loading custom assets.)*
```usd
#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def Xform "Environments"
    {
    }

    def Xform "Looks"
    {
    }

    def Xform "Physics"
    {
    }

    def Xform "Simulation"
    {
    }

    def Xform "Robots"
    {
        def "Humanoid" (
            assetPaths = ["omniverse://localhost/NVIDIA/Assets/ISAAC/2023.1.1/Isaac/Robots/Humanoid/franka_humanoid_with_hands.usd"] # Example path
        )
        {
        }
    }
}
```
*(This USD is a minimal example of how an existing asset might be referenced. In practice, a full USD scene would be generated or edited within Isaac Sim's GUI or Python API.)*

## 5. Generating Synthetic Data

Isaac Sim's powerful RTX renderer enables data generation that closely mimics real-world sensors. You can attach various sensors (RGB, Depth, LiDAR, Semantic Segmentation, etc.) to your robot and capture data at high fidelity.

Here's a conceptual Python script to generate synthetic data. This script would be run from within Isaac Sim's scripting environment or as an extension.

**File: `code-examples/module3-isaac-ai-brain/chapter3.1/synthetic_data_gen.py`**
```python
import omni.isaac.core.utils.nucleus as nucleus_utils
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, add_reference_to_stage
from omni.isaac.sensor import Camera, LidarRtx

# Initialize Isaac Sim world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Define paths to assets
ASSET_ROOT_PATH = nucleus_utils.get_nucleus_assets_path()
humanoid_asset_path = f"{ASSET_ROOT_PATH}/NVIDIA/Assets/ISAAC/2023.1.1/Isaac/Robots/Humanoid/franka_humanoid_with_hands.usd"

# Create a new stage (optional, if you're not loading an existing one)
create_new_stage()

# Add the humanoid robot to the stage
add_reference_to_stage(usd_path=humanoid_asset_path, prim_path="/World/Humanoid")

# --- Add Sensors to the Humanoid (conceptual) ---
# This is a high-level conceptual example. In a real script, you'd attach sensors
# to specific prims of the humanoid and configure them.

# Add an RGB Camera
camera_prim_path = "/World/Humanoid/Camera"
camera = Camera(
    prim_path=camera_prim_path,
    position=omni.isaac.core.utils.numpy.array([0.0, 0.0, 1.0]), # Example position relative to humanoid
    orientation=omni.isaac.core.utils.numpy.array([0.5, 0.5, -0.5, -0.5]), # Example orientation
    resolution=(640, 480),
    fov_y=60.0,
)
camera.set_render_product_path(f"{camera_prim_path}/RenderProduct") # Assign a render product

# Add an RTX LiDAR
lidar_prim_path = "/World/Humanoid/Lidar"
lidar = LidarRtx(
    prim_path=lidar_prim_path,
    position=omni.isaac.core.utils.numpy.array([0.0, 0.0, 1.2]), # Example position
    orientation=omni.isaac.core.utils.numpy.array([0.5, 0.5, -0.5, -0.5]), # Example orientation
)
lidar.set_resolution(0.4)
lidar.set_rotation_rate(10.0)
lidar.set_fov(360.0)

# Start simulation
world.reset()
world.run_simulation()

# --- Data Generation Loop (conceptual) ---
for i in range(100): # Generate 100 frames
    world.step(render=True)
    # --- Capture and Save Data (conceptual) ---
    # In a full implementation, you'd use the sensor APIs to get data
    # and save it to files (e.g., PNG for images, PLY/PCAP for point clouds).
    #
    # Example (highly conceptual, not runnable without full Isaac Sim API context):
    # rgb_data = camera.get_data().get_image_data()
    # depth_data = camera.get_depth_data().get_image_data()
    # lidar_points = lidar.get_point_cloud_data()
    #
    # Save data to disk (e.g., using PIL for images, numpy/open3d for point clouds)
    #
    print(f"Generated data for frame {i}")

print("Synthetic data generation complete.")
# The Isaac Sim GUI will remain open after script execution.
# To exit gracefully: omni.isaac.app.app.shutdown()
# This script would typically be run within Isaac Sim's Python environment.

import omni.isaac.core.utils.nucleus as nucleus_utils
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, add_reference_to_stage
from omni.isaac.sensor import Camera, LidarRtx
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import os

# Initialize Isaac Sim world
world = World(stage_units_in_meters=1.0)

# Define output directory for synthetic data
output_dir = os.path.join(os.getcwd(), "synthetic_data")
os.makedirs(output_dir, exist_ok=True)

# Define paths to assets
ASSET_ROOT_PATH = nucleus_utils.get_nucleus_assets_path()
humanoid_asset_path = f"{ASSET_ROOT_PATH}/NVIDIA/Assets/ISAAC/2023.1.1/Isaac/Robots/Humanoid/franka_humanoid_with_hands.usd"
ground_asset_path = f"{ASSET_ROOT_PATH}/NVIDIA/Assets/Scenes/Templates/Basic/simple_room.usd"

# Create a new stage and add a ground plane
create_new_stage()
add_reference_to_stage(usd_path=ground_asset_path, prim_path="/World/GroundPlane")

# Add the humanoid robot to the stage
humanoid_prim_path = "/World/Humanoid"
add_reference_to_stage(usd_path=humanoid_asset_path, prim_path=humanoid_prim_path)

# --- Add Sensors to the Humanoid ---
# Add an RGB Camera
camera_prim_path = f"{humanoid_prim_path}/Camera"
camera = Camera(
    prim_path=camera_prim_path,
    position=np.array([0.0, 0.0, 1.0]), # Example position relative to humanoid's base
    orientation=np.array([0.5, 0.5, -0.5, -0.5]), # Example orientation
    resolution=(640, 480),
    fov_y=60.0,
)

# Add an RTX LiDAR
lidar_prim_path = f"{humanoid_prim_path}/Lidar"
lidar = LidarRtx(
    prim_path=lidar_prim_path,
    position=np.array([0.0, 0.0, 1.2]), # Example position
    orientation=np.array([0.5, 0.5, -0.5, -0.5]), # Example orientation
)
lidar.set_resolution(0.4)
lidar.set_rotation_rate(10.0)
lidar.set_fov(360.0)

# Create Synthetic Data Helper
sd_helper = SyntheticDataHelper()
sd_helper.initialize(sensor_names=[str(camera.prim_path), str(lidar.prim_path)])


# Start simulation
world.reset()
world.run_simulation()

# --- Data Generation Loop ---
num_frames = 10
for i in range(num_frames):
    world.step(render=True)
    
    # Randomize humanoid pose for data diversity (conceptual, requires more complex control)
    # Here, we just step through the simulation.
    
    # Capture and Save Data
    rp_camera = sd_helper.get_breaking_pointcloud_data(str(camera.prim_path))
    rp_lidar = sd_helper.get_lidar_data(str(lidar.prim_path)) # Adjusted for LidarRtx

    # Save RGB Image
    camera_rgb_data = camera.get_data().get_image_data()
    if camera_rgb_data is not None:
        omni.isaac.core.utils.image.save_image(
            camera_rgb_data, 
            os.path.join(output_dir, f"rgb_frame_{i:04d}.png")
        )

    # Save Lidar Data (example: save as PLY)
    if rp_lidar is not None:
        points = rp_lidar["point_cloud_data"]
        # Save point cloud data (e.g., using open3d if installed, or simply numpy.savetxt)
        np.savetxt(os.path.join(output_dir, f"lidar_frame_{i:04d}.txt"), points)


    print(f"Generated data for frame {i}")

print(f"Synthetic data generation complete. Saved to {output_dir}")
# The Isaac Sim GUI will remain open after script execution.
# To exit gracefully: omni.isaac.app.app.shutdown()
# This script would typically be run within Isaac Sim's Python environment.

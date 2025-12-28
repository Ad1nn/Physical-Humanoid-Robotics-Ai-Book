# Isaac Sim Setup Guide

This directory contains resources and instructions for setting up your NVIDIA Isaac Sim environment for Chapter 3.1.

## Important Prerequisites

-   **NVIDIA GPU**: Required with 8GB+ VRAM.
-   **NVIDIA Drivers**: Latest proprietary drivers installed.
-   **Docker & NVIDIA Container Toolkit**: Properly installed and configured.

## Step 1: Download Isaac Sim

NVIDIA Isaac Sim is distributed via the NVIDIA Developer Zone.

1.  Go to the [NVIDIA Developer website](https://developer.nvidia.com/isaac-sim).
2.  Download the latest recommended version (e.g., 2023.1.1 or later). This usually involves downloading a tar.gz archive.
3.  Extract the archive to a suitable location on your system.

## Step 2: Launch Isaac Sim via Docker

Isaac Sim uses Docker containers for its execution, ensuring a consistent environment.

1.  Navigate to the extracted Isaac Sim directory.
2.  Locate the `run_isaac_sim.sh` script.
3.  Execute the script:
    ```bash
    ./run_isaac_sim.sh
    ```
    This script will handle pulling the necessary Docker image (`nvcr.io/nvidia/isaac-sim:2023.1.1` or similar) and launching the container with appropriate GPU and display configurations.

## Step 3: Basic Scene Verification

Once Isaac Sim launches:

1.  Open the Isaac Sim UI.
2.  Load one of the example scenes (e.g., a simple `Franka` robot or a `Humanoid` scene).
3.  Ensure you can interact with the scene (e.g., move the camera, play/pause simulation).

For detailed setup and troubleshooting, refer to the [official Isaac Sim documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html).

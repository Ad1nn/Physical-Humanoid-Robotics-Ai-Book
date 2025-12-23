# Research & Decisions: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

This document records the key technical research and decisions made during the planning phase for Module 3.

## 1. Isaac Sim Deployment Approach

-   **Topic**: How students will deploy and run NVIDIA Isaac Sim.
-   **Decision**: Emphasize Docker-based deployment using NVIDIA Container Toolkit.
-   **Rationale**: Isaac Sim has significant hardware requirements (NVIDIA GPU, specific drivers). A Docker-based approach ensures a consistent and reproducible environment, minimizing setup headaches across diverse student machines (even if they have different Linux distributions or slightly different driver versions). This aligns with the "Simulation-First Approach" and "Technical Accuracy" principles.
-   **Alternatives Considered**: Direct native installation on Ubuntu. This is prone to version conflicts and driver issues, making it less suitable for a broad audience.

## 2. VSLAM Implementation Depth

-   **Topic**: The level of detail and complexity for VSLAM implementation using Isaac ROS.
-   **Decision**: Focus on integrating existing Isaac ROS VSLAM packages (e.g., `visual_slam`) rather than implementing VSLAM algorithms from scratch.
-   **Rationale**: The goal is to teach students how to *use* state-of-the-art, hardware-accelerated VSLAM, not to be a deep dive into VSLAM algorithm development. This aligns with the "Practical-First Pedagogy" by providing functional pipelines. The "Hardware acceleration" aspect of Isaac ROS is a key learning point.
-   **Alternatives Considered**: Presenting a simplified VSLAM algorithm for educational purposes. This would detract from the core learning objective of using Isaac ROS.

## 3. Nav2 Algorithm Coverage

-   **Topic**: Which Nav2 path planning algorithms to cover for humanoid robots.
-   **Decision**: Focus on DWA (Dynamic Window Approach) and TEB (Timed Elastic Band) as primary examples, with mention of Regulated Pure Pursuit.
-   **Rationale**: DWA and TEB are widely used and provide a good contrast in their approach to local planning. DWA is often easier to understand initially, while TEB offers more sophisticated trajectory optimization. Regulated Pure Pursuit is a common global planner. This selection balances practical utility with conceptual understanding.
-   **Alternatives Considered**: Covering all available Nav2 planners in depth. This would lead to information overload and dilute the focus.

## 4. Synthetic Data Emphasis Level

-   **Topic**: How much emphasis to place on synthetic data generation versus using pre-recorded datasets.
-   **Decision**: Strong emphasis on students generating their *own* synthetic data using Isaac Sim's tools.
-   **Rationale**: A key value proposition of Isaac Sim is its synthetic data generation capabilities. Empowering students to create custom datasets is fundamental to training perception models for novel robotic tasks.
-   **Alternatives Considered**: Relying solely on pre-existing synthetic datasets. This would diminish the hands-on learning experience and miss a core Isaac Sim feature.

## 5. Hardware Requirements Threshold

-   **Topic**: Specific hardware recommendations for running Isaac Sim and Isaac ROS.
-   **Decision**: Minimum NVIDIA GPU 8GB+ VRAM.
-   **Rationale**: Isaac Sim and Isaac ROS are highly GPU-accelerated. Lower VRAM can lead to significant performance degradation or inability to run complex simulations. This directly aligns with the "Technical Constraints" in the Constitution and the user's explicit requirement.
-   **Alternatives Considered**: Recommending lower-end GPUs. This would lead to frustrating user experiences and non-functional examples.

## 6. Code Placement Strategy

-   **Topic**: Where to place code examples within the repository structure.
-   **Decision**: Code examples will be placed in `code-examples/module3-isaac-ai-brain/` following a chapter-specific subdirectory structure, separate from the Docusaurus `docs`.
-   **Rationale**: This maintains consistency with Modules 1 and 2 and adheres to the "Repository Structure" defined in the Constitution. It keeps documentation clean and code runnable.

## 7. Isaac ROS Package Selection

-   **Topic**: Which specific Isaac ROS packages to highlight and use for VSLAM and other functionalities.
-   **Decision**: Primarily use `visual_slam` for VSLAM, `stereo_image_proc` for stereo depth, and any necessary `ros_image_pipeline` components for image processing.
-   **Rationale**: These packages directly address the VSLAM and perception goals of the module and are core to Isaac ROS capabilities. They provide hardware-accelerated implementations suitable for the target platform.
-   **Alternatives Considered**: Using non-Isaac ROS equivalents. This would defeat the purpose of teaching NVIDIA Isaac's advantages.

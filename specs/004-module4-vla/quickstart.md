# Quickstart: Module 4 Environment Setup - VLA Components

This guide provides essential steps to configure your development environment for the Module 4 "Vision-Language-Action (VLA)" hands-on examples. This module relies heavily on Large Language Models (LLMs) via API access and Speech-to-Text (STT) models like OpenAI Whisper.

## Prerequisites

1.  **Ubuntu 22.04 LTS**: Or a Docker container running it.
2.  **NVIDIA GPU**: Required if you plan to run local Whisper models or any local GPU-accelerated perception.
3.  **ROS 2 Humble**: Fully installed and sourced.
4.  **Python 3.10+**: With `pip` for package management.
5.  **Module 1-3 Completion**: Understanding of ROS 2 basics, simulation (Gazebo, Isaac Sim), and Nav2.
6.  **Internet Access**: Required for LLM API calls and downloading Whisper models.

## 1. OpenAI Whisper Integration

You can integrate Whisper locally or via an API. For this module, we will focus on basic setup.

### Option A: Local Whisper Installation (Recommended for development)

1.  **Install `ffmpeg`**: Required for audio processing.
    ```bash
    sudo apt update
    sudo apt install ffmpeg
    ```
2.  **Install OpenAI Whisper Python package**:
    ```bash
    pip install openai-whisper
    ```
3.  **Download Whisper Model**: The first time you run Whisper, it will download a model (e.g., `base.en`). Ensure you have sufficient disk space.

### Option B: Whisper API Access (if using OpenAI services)

1.  **Obtain OpenAI API Key**: Register on the OpenAI platform and obtain an API key.
2.  **Install OpenAI Python library**:
    ```bash
    pip install openai
    ```
3.  **Set Environment Variable**:
    ```bash
    export OPENAI_API_KEY="YOUR_API_KEY"
    ```

## 2. Large Language Model (LLM) API Access

For cognitive planning, you will interact with LLMs via their APIs.

### Option A: OpenAI GPT-4 API

1.  **Obtain OpenAI API Key**: (Same as for Whisper API, if using OpenAI).
2.  **Install OpenAI Python library**:
    ```bash
    pip install openai
    ```
3.  **Set Environment Variable**:
    ```bash
    export OPENAI_API_KEY="YOUR_API_KEY"
    ```

### Option B: Anthropic Claude API

1.  **Obtain Anthropic API Key**: Register on the Anthropic platform.
2.  **Install Anthropic Python library**:
    ```bash
    pip install anthropic
    ```
3.  **Set Environment Variable**:
    ```bash
    export ANTHROPIC_API_KEY="YOUR_API_KEY"
    ```

### Option C: Google Gemini API

1.  **Obtain Google API Key**: Follow instructions for Google AI Studio or Google Cloud.
2.  **Install Google Generative AI Python library**:
    ```bash
    pip install google-generativeai
    ```
3.  **Set Environment Variable**:
    ```bash
    export GOOGLE_API_KEY="YOUR_API_KEY"
    ```

## 3. ROS 2 VLA Integration Setup

You will create custom ROS 2 packages to integrate these components.

1.  **Create a ROS 2 Workspace**:
    ```bash
    mkdir -p ~/vla_ros_ws/src
    cd ~/vla_ros_ws/src
    ```
2.  **Clone Relevant Repositories**: (e.g., any community-contributed Whisper or LLM ROS 2 packages, or your own).
3.  **Build and Source**:
    ```bash
    cd ~/vla_ros_ws
    colcon build --merge-install
    source install/setup.bash
    ```

## 4. Clone the Book's Code Repository

If you haven't already, clone the repository containing the code examples.

```bash
git clone <repository_url>
cd physical-ai-book/code-examples/module4-vla
```

You are now ready to begin integrating vision, language, and action for your autonomous humanoid!

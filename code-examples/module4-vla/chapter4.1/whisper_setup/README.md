# OpenAI Whisper Setup Guide

This directory contains resources and instructions for setting up OpenAI Whisper for speech-to-text functionality, as discussed in Chapter 4.1.

## Important Prerequisites

-   **Python 3.10+**: Your development environment should have a compatible Python version.
-   **`pip`**: Python package installer.
-   **`ffmpeg`**: Required for audio processing by Whisper.

## Step 1: Install `ffmpeg`

Whisper relies on `ffmpeg` to handle audio files.

```bash
sudo apt update
sudo apt install ffmpeg
```

## Step 2: Install OpenAI Whisper Python Package

Install the official Whisper package using pip:

```bash
pip install openai-whisper
```

## Step 3: Download Whisper Model

When you first use Whisper, it will automatically download a language model. You can specify which model to use. Larger models offer better accuracy but require more resources.

Example usage (Python):

```python
import whisper

# Load a base English-only model
model = whisper.load_model("base.en")

# Transcribe an audio file
result = model.transcribe("audio.mp3")
print(result["text"])
```

For detailed usage, model options, and troubleshooting, refer to the [official OpenAI Whisper GitHub repository](https://github.com/openai/whisper).

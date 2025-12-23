# LLM API Access Setup Guide

This directory contains resources and instructions for setting up API access to Large Language Models (LLMs) for cognitive planning, as discussed in Chapter 4.2.

## Important Prerequisites

-   **Internet Access**: Essential for communicating with LLM APIs.
-   **API Keys**: You will need to obtain API keys from your chosen LLM provider(s). **Never commit API keys directly to your code or public repositories.** Use environment variables for secure storage.
-   **Python 3.10+**: Your development environment should have a compatible Python version.

## Step 1: Obtain API Keys

Choose your preferred LLM provider(s) and follow their instructions to obtain an API key:

-   **OpenAI**: [https://platform.openai.com/](https://platform.openai.com/)
-   **Anthropic (Claude)**: [https://www.anthropic.com/api](https://www.anthropic.com/api)
-   **Google (Gemini)**: [https://ai.google.dev/](https://ai.google.dev/)

## Step 2: Install Python Libraries

Install the Python client library for your chosen LLM provider(s):

-   **For OpenAI (GPT-4, etc.)**:
    ```bash
    pip install openai
    ```
-   **For Anthropic (Claude)**:
    ```bash
    pip install anthropic
    ```
-   **For Google (Gemini)**:
    ```bash
    pip install google-generativeai
    ```

## Step 3: Securely Store API Keys (Environment Variables)

To avoid hardcoding your API keys, store them as environment variables.

### On Linux/macOS:

Add the following lines to your shell's profile file (e.g., `~/.bashrc`, `~/.zshrc`) and then `source` the file:

```bash
export OPENAI_API_KEY="sk-YOUR_OPENAI_API_KEY"
export ANTHROPIC_API_KEY="sk-YOUR_ANTHROPIC_API_KEY"
export GOOGLE_API_KEY="YOUR_GOOGLE_API_KEY"
```

### On Windows (PowerShell):

```powershell
$env:OPENAI_API_KEY="sk-YOUR_OPENAI_API_KEY"
$env:ANTHROPIC_API_KEY="sk-YOUR_ANTHROPIC_API_KEY"
$env:GOOGLE_API_KEY="YOUR_GOOGLE_API_KEY"
```
*(Note: For permanent changes, you might need to use System Environment Variables settings.)*

## Step 4: Example Python Usage

After setting your API key(s) as environment variables, you can use the respective client libraries in Python.

### OpenAI Example:

```python
import os
from openai import OpenAI

client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

response = client.chat.completions.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": "Tell me a joke about robots."},
    ]
)
print(response.choices[0].message.content)
```

For detailed usage, model options, and troubleshooting, refer to the [official documentation of your chosen LLM provider](https://platform.openai.com/docs/, https://docs.anthropic.com/claude/reference/getting-started-with-the-api, https://ai.google.dev/docs).

# Quickstart Guide: Vision-Language-Action (VLA) for Humanoid Robotics

**Feature**: Vision-Language-Action (VLA) for Humanoid Robotics
**Created**: 2025-12-24
**Spec**: [specs/4-vla-education/spec.md](specs/4-vla-education/spec.md)

## Overview

This quickstart guide provides students with the essential setup instructions to begin learning about Vision-Language-Action (VLA) systems for humanoid robotics. This guide assumes familiarity with ROS 2 and basic LLM concepts.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or equivalent Linux distribution
- Python 3.8 or higher
- Node.js 18+ and npm 8+
- At least 8GB RAM (16GB recommended for vision processing)
- GPU with CUDA support recommended (for vision processing, optional)
- Internet connection for API access

### Software Dependencies
- ROS 2 Humble Hawksbill (or newer)
- Docker (for containerized examples)
- Git version control
- OpenAI API key (for Whisper and LLM examples)
- Docusaurus development environment

## Environment Setup

### 1. ROS 2 Installation
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Python Environment Setup
```bash
# Create virtual environment
python3 -m venv vla_env
source vla_env/bin/activate
pip install --upgrade pip

# Install required Python packages
pip install rclpy openai torch torchvision torchaudio transformers datasets whisper speechrecognition numpy matplotlib pillow
```

### 3. OpenAI API Configuration
```bash
# Create .env file for API keys
echo "OPENAI_API_KEY=your_api_key_here" > .env

# Install python-dotenv to manage environment variables
pip install python-dotenv
```

### 4. Docusaurus Development Setup
```bash
# Install Node.js dependencies
npm install
npm install @docusaurus/module-type-aliases @docusaurus/types

# Install additional packages for VLA examples
npm install @lexical/react lexical @docusaurus/preset-classic
```

## Repository Structure

After setup, your repository should look like this:

```
humanoid-ai-robotics-book/
├── docs/
│   └── module-4/
│       ├── intro-to-vla.md
│       ├── voice-to-action-whisper.md
│       └── cognitive-planning-llms.md
├── src/
│   └── components/
├── specs/
│   └── 4-vla-education/
├── package.json
├── docusaurus.config.js
└── .env (not committed to version control)
```

## Running the Docusaurus Documentation

### 1. Start Local Development Server
```bash
# Navigate to project root
cd humanoid-ai-robotics-book

# Start development server
npm start
```

### 2. Access Documentation
Open your browser to `http://localhost:3000` to view the documentation. The Module 4 content will be available under the "Vision-Language-Action" section.

## Basic VLA System Example

### 1. Voice Command Processing
```python
# Example: Basic voice command processing with Whisper
import openai
import speech_recognition as sr
from dotenv import load_dotenv
import os

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")

def listen_and_process():
    # Initialize recognizer
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for command...")
        audio = r.listen(source)

    # Convert speech to text using Whisper
    try:
        text = r.recognize_whisper_api(audio, api_key=openai.api_key)
        print(f"Recognized: {text}")
        return text
    except Exception as e:
        print(f"Error: {e}")
        return None
```

### 2. Simple Intent Extraction
```python
# Example: Basic intent extraction using OpenAI API
def extract_intent(text_command):
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'."},
            {"role": "user", "content": text_command}
        ],
        temperature=0.1
    )

    import json
    return json.loads(response.choices[0].message.content)
```

### 3. ROS 2 Action Client Example
```python
# Example: ROS 2 action client for navigation
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient:
    def __init__(self):
        self.node = rclpy.create_node('navigation_client')
        self._action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
```

## Running VLA Examples

### 1. Voice-to-Action Example
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the voice-to-action node
python3 examples/vla_voice_to_action.py
```

### 2. Vision Processing Example
```bash
# Run vision processing example
python3 examples/vla_vision_processing.py
```

### 3. End-to-End VLA Example
```bash
# Run complete VLA system
python3 examples/vla_complete_system.py
```

## Common Issues and Troubleshooting

### 1. OpenAI API Access
- Ensure your API key is correctly set in `.env`
- Check your OpenAI account billing status
- Verify internet connectivity

### 2. ROS 2 Environment
- Always source ROS 2 environment before running commands
- Check that ROS 2 packages are properly installed
- Ensure correct ROS 2 distribution is sourced

### 3. Audio Input Issues
- Verify microphone permissions
- Check audio input device configuration
- Test with `arecord -l` to list audio devices

### 4. Python Package Conflicts
- Use virtual environments to avoid conflicts
- Ensure all packages are compatible with your Python version
- Check CUDA compatibility if using GPU acceleration

## Next Steps

1. Complete the **Vision-Language-Action Overview** chapter to understand fundamental concepts
2. Explore the **Voice-to-Action** chapter to implement Whisper-based voice command processing
3. Study the **Cognitive Planning with LLMs** chapter to learn about natural language goal translation
4. Try the hands-on exercises in each chapter
5. Work on the Autonomous Humanoid capstone project

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [Docusaurus Documentation](https://docusaurus.io/)
- [Vision-Language Models Survey](https://arxiv.org/abs/2302.08275)
- [Robotics Transformer Paper](https://arxiv.org/abs/2208.11721)
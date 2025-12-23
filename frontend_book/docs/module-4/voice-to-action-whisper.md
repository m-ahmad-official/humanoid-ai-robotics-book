---
sidebar_position: 2
title: "Voice-to-Action Systems with OpenAI Whisper"
---

# Voice-to-Action Systems with OpenAI Whisper

## Introduction

Voice-to-action systems enable natural human-robot interaction by converting spoken language into executable robot commands. These systems are crucial for humanoid robotics, as they allow humans to communicate with robots using their natural language. OpenAI Whisper provides a powerful automatic speech recognition (ASR) capability that can be integrated into voice-to-action pipelines.

This chapter explores how to implement voice-to-action systems using OpenAI Whisper, covering the complete pipeline from audio capture to robot command execution.

## Understanding Automatic Speech Recognition (ASR) Systems

Automatic Speech Recognition (ASR) systems convert spoken language into text. These systems are fundamental to voice-to-action applications, as they bridge the gap between human speech and machine-understandable commands.

### Core Components of ASR Systems

#### 1. Audio Preprocessing
- **Noise Reduction**: Filtering out background noise to improve recognition accuracy
- **Normalization**: Adjusting audio levels and format for consistent processing
- **Feature Extraction**: Converting audio signals into features suitable for neural networks

#### 2. Acoustic Models
- **Signal Processing**: Converting audio waveforms into spectrograms or other feature representations
- **Phoneme Recognition**: Identifying basic speech sounds that make up words
- **Language Modeling**: Using context to improve recognition accuracy

#### 3. Language Models
- **Grammar Integration**: Understanding syntactic structures to improve recognition
- **Context Awareness**: Using surrounding words to disambiguate similar-sounding words
- **Domain Adaptation**: Optimizing for specific vocabularies or speaking styles

### Types of ASR Systems

#### Cloud-Based ASR
- **Advantages**: High accuracy, multilingual support, continuous updates
- **Disadvantages**: Requires internet connection, potential privacy concerns, API costs
- **Examples**: OpenAI Whisper API, Google Speech-to-Text, AWS Transcribe

#### On-Premise ASR
- **Advantages**: Better privacy control, no internet dependency, customizable
- **Disadvantages**: Higher computational requirements, maintenance overhead
- **Examples**: Self-hosted Whisper, DeepSpeech, wav2vec 2.0

#### Hybrid Approaches
- **Advantages**: Balance between privacy and accuracy, fallback capabilities
- **Disadvantages**: More complex architecture, potential consistency issues

## Understanding OpenAI Whisper

OpenAI Whisper is a robust automatic speech recognition (ASR) system trained on diverse datasets. It offers several advantages for voice-to-action systems:

- **Multilingual Support**: Supports 98+ languages
- **Robustness**: Handles accents, background noise, and technical speech
- **Accuracy**: Strong performance across various audio conditions
- **Accessibility**: Available through API or self-hosted models

### Whisper Architecture

Whisper uses a multi-task approach that handles:
- Speech recognition
- Language identification
- Translation (for some models)
- Speech segmentation

The model is particularly effective for voice-to-action systems because it can handle diverse speaking styles and environmental conditions.

### Whisper Model Variants

Whisper comes in several sizes with different performance characteristics:

| Model | Size | Languages | Required VRAM | Relative Speed |
|-------|------|-----------|---------------|----------------|
| tiny  | 39M  | ~100      | 1GB           | 32x            |
| base  | 74M  | ~100      | 1GB           | 16x            |
| small | 244M | ~100      | 2GB           | 6x             |
| medium| 769M | ~100      | 5GB           | 2x             |
| large | 1550M| ~100      | 10GB          | 1x             |

For voice-to-action applications, the choice depends on:
- **Real-time requirements**: Smaller models for faster processing
- **Accuracy needs**: Larger models for better recognition
- **Resource constraints**: Available computational resources
- **Privacy requirements**: Self-hosting vs. cloud API considerations

### Whisper in Voice-to-Action Context

In voice-to-action systems, Whisper serves as the first step in the pipeline:

```
[Raw Audio] → [Whisper ASR] → [Transcribed Text] → [Intent Extraction] → [Action Mapping]
```

The quality of Whisper's transcription directly impacts the subsequent intent extraction and action mapping steps.

## Voice Command Processing Pipeline

The voice-to-action pipeline consists of several key stages:

### 1. Audio Capture and Preprocessing

The first step in any voice-to-action system is capturing audio input:

```python
import speech_recognition as sr

def capture_audio():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for command...")
        # Adjust for ambient noise
        r.adjust_for_ambient_noise(source)
        # Listen for audio
        audio = r.listen(source)
        return audio
```

### 2. Speech Recognition with Whisper

Once audio is captured, it's processed using Whisper to convert speech to text:

```python
import openai
from dotenv import load_dotenv
import os

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_with_whisper(audio):
    try:
        # Convert audio to the format expected by Whisper API
        audio_data = audio.get_wav_data()

        # Use Whisper API for transcription
        response = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_data
        )

        return response.text
    except Exception as e:
        print(f"Error transcribing audio: {e}")
        return None
```

### 3. Intent Extraction

After transcription, the text must be processed to extract the intended action:

```python
def extract_intent(text_command):
    prompt = f"""
    You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'.

    Command: {text_command}

    Examples:
    - Command: "Go to the kitchen" → {{"action_type": "NAVIGATE", "parameters": {{"target_location": "kitchen"}}}}
    - Command: "Pick up the red cup" → {{"action_type": "GRASP", "parameters": {{"target_object": "red cup"}}}}
    - Command: "Stop moving" → {{"action_type": "STOP", "parameters": {{}}}}
    """

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.1
    )

    import json
    try:
        return json.loads(response.choices[0].message.content)
    except json.JSONDecodeError:
        # If JSON parsing fails, return a default action
        return {"action_type": "UNKNOWN", "parameters": {}}
```

### 4. Command Mapping

The extracted intent must be mapped to specific robot actions:

```python
def map_to_robot_action(intent):
    action_type = intent.get("action_type", "UNKNOWN")
    parameters = intent.get("parameters", {})

    if action_type == "NAVIGATE":
        target_location = parameters.get("target_location", "")
        return create_navigation_command(target_location)
    elif action_type == "GRASP":
        target_object = parameters.get("target_object", "")
        return create_grasp_command(target_object)
    elif action_type == "STOP":
        return create_stop_command()
    else:
        return create_unknown_command()
```

## Implementation Example

Here's a complete example of a voice-to-action system:

```python
import openai
import speech_recognition as sr
from dotenv import load_dotenv
import os
import json
from datetime import datetime

class VoiceToActionSystem:
    def __init__(self):
        load_dotenv()
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.recognizer = sr.Recognizer()

    def listen_and_process(self):
        # Capture audio
        with sr.Microphone() as source:
            print("Listening for command...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        # Transcribe with Whisper
        try:
            # Convert to WAV for Whisper API
            wav_data = audio.get_wav_data()

            # Use Whisper API
            transcription = openai.Audio.transcribe(
                model="whisper-1",
                file=wav_data
            )

            text = transcription.text
            print(f"Recognized: {text}")

            # Extract intent
            intent = self.extract_intent(text)
            print(f"Intent: {intent}")

            # Map to robot action
            robot_command = self.map_to_robot_action(intent)
            return robot_command

        except Exception as e:
            print(f"Error processing voice command: {e}")
            return None

    def extract_intent(self, text_command):
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'."},
                {"role": "user", "content": text_command}
            ],
            temperature=0.1
        )

        try:
            return json.loads(response.choices[0].message.content)
        except json.JSONDecodeError:
            return {"action_type": "UNKNOWN", "parameters": {}}

    def map_to_robot_action(self, intent):
        # This would interface with your robot's action system
        # For this example, we'll just return the command structure
        return {
            "type": "robot_action",
            "intent": intent,
            "timestamp": str(datetime.now())
        }

# Usage example
if __name__ == "__main__":
    vta_system = VoiceToActionSystem()
    command = vta_system.listen_and_process()
    if command:
        print(f"Executing command: {command}")
        # Execute the command on the robot
        # execute_robot_command(command)
```

### Advanced Whisper Integration Example

Here's a more advanced example that includes error handling, confidence scoring, and integration with ROS 2:

```python
import openai
import speech_recognition as sr
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import time
from typing import Optional, Dict, Any

class AdvancedVoiceToActionSystem:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('voice_to_action_node')

        # Publishers for robot commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set up audio parameters
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        # Set up OpenAI API
        self.openai_api_key = rospy.get_param('~openai_api_key', os.getenv('OPENAI_API_KEY'))
        openai.api_key = self.openai_api_key

        # Confidence threshold for voice commands
        self.confidence_threshold = 0.8

    def process_voice_command(self) -> Optional[Dict[str, Any]]:
        """
        Process a single voice command with error handling
        """
        try:
            # Listen for audio
            with self.microphone as source:
                rospy.loginfo("Listening for voice command...")
                audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0)

            # Transcribe with Whisper
            transcription = self.transcribe_with_whisper(audio)
            if not transcription or transcription['confidence'] < self.confidence_threshold:
                rospy.logwarn(f"Low confidence transcription: {transcription}")
                return None

            # Extract intent with LLM
            intent = self.extract_intent(transcription['text'])
            if not intent:
                rospy.logwarn("Could not extract intent from transcription")
                return None

            # Execute the action
            success = self.execute_robot_action(intent)
            if success:
                rospy.loginfo(f"Successfully executed action: {intent}")
                return intent
            else:
                rospy.logerr(f"Failed to execute action: {intent}")
                return None

        except sr.WaitTimeoutError:
            rospy.logwarn("No speech detected within timeout period")
            return None
        except sr.UnknownValueError:
            rospy.logwarn("Whisper could not understand the audio")
            return None
        except Exception as e:
            rospy.logerr(f"Error processing voice command: {e}")
            return None

    def transcribe_with_whisper(self, audio) -> Optional[Dict[str, Any]]:
        """
        Transcribe audio using OpenAI Whisper with confidence estimation
        """
        try:
            # Convert audio to required format
            wav_data = audio.get_wav_data()

            # Use Whisper API
            response = openai.Audio.transcribe(
                model="whisper-1",
                file=wav_data,
                response_format="verbose_json",
                timestamp_granularities=["segment"]
            )

            # Estimate confidence based on the transcription
            # Note: Whisper doesn't return explicit confidence scores, so we estimate based on other factors
            text = response.text
            duration = response.duration
            segments = response.segments

            # Simple confidence estimation (in a real system, you'd use more sophisticated methods)
            confidence = self.estimate_transcription_confidence(text, segments)

            return {
                "text": text,
                "duration": duration,
                "segments": segments,
                "confidence": confidence
            }

        except Exception as e:
            rospy.logerr(f"Error in Whisper transcription: {e}")
            return None

    def estimate_transcription_confidence(self, text: str, segments: list) -> float:
        """
        Estimate confidence of transcription based on various factors
        """
        if not text or len(text.strip()) == 0:
            return 0.0

        # Simple heuristics for confidence estimation
        confidence = 1.0

        # Penalize very short transcriptions (might be noise)
        if len(text.strip()) < 3:
            confidence *= 0.3

        # Consider number of segments (more segments might indicate better quality)
        if len(segments) > 0:
            confidence *= min(1.0, len(segments) * 0.1)

        # Return clamped confidence value
        return max(0.0, min(1.0, confidence))

    def extract_intent(self, text_command: str) -> Optional[Dict[str, Any]]:
        """
        Extract intent from text command using LLM
        """
        try:
            prompt = f"""
            You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'.

            The possible action types are:
            - NAVIGATE: For movement commands
            - GRASP: For object manipulation
            - SPEAK: For verbal responses
            - STOP: For stopping current actions
            - FOLLOW: For following a person or object

            Command: {text_command}

            Respond with a JSON object containing 'action_type' and 'parameters'.
            """

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1
            )

            # Parse the response
            content = response.choices[0].message.content.strip()

            # Remove any markdown formatting if present
            if content.startswith("```json"):
                content = content[7:]  # Remove ```json
                if content.endswith("```"):
                    content = content[:-3]  # Remove ```

            return json.loads(content)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing LLM response: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"Error extracting intent: {e}")
            return None

    def execute_robot_action(self, intent: Dict[str, Any]) -> bool:
        """
        Execute robot action based on intent
        """
        action_type = intent.get('action_type', 'UNKNOWN')
        parameters = intent.get('parameters', {})

        if action_type == 'NAVIGATE':
            return self.execute_navigation(parameters)
        elif action_type == 'SPEAK':
            return self.execute_speak(parameters)
        elif action_type == 'STOP':
            return self.execute_stop(parameters)
        else:
            rospy.logwarn(f"Unknown action type: {action_type}")
            return False

    def execute_navigation(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute navigation command
        """
        try:
            # Example navigation command
            target_location = parameters.get('target_location', 'unknown')

            # In a real system, you would send this to a navigation stack
            cmd = Twist()

            # Simple example: move forward for demonstration
            if target_location in ['forward', 'ahead', 'go']:
                cmd.linear.x = 0.5  # Move forward at 0.5 m/s
            elif target_location in ['back', 'backward']:
                cmd.linear.x = -0.5  # Move backward
            elif target_location in ['left', 'turn left']:
                cmd.angular.z = 0.5  # Turn left
            elif target_location in ['right', 'turn right']:
                cmd.angular.z = -0.5  # Turn right
            else:
                rospy.loginfo(f"Unknown navigation target: {target_location}")
                return False

            # Publish the command
            self.cmd_vel_pub.publish(cmd)
            time.sleep(2)  # Execute for 2 seconds

            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            return True

        except Exception as e:
            rospy.logerr(f"Error executing navigation: {e}")
            return False

    def execute_speak(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute speak command
        """
        try:
            text = parameters.get('text', 'Hello')
            rospy.loginfo(f"Speaking: {text}")

            # In a real system, you would publish to a text-to-speech system
            # For this example, we just log the text
            return True

        except Exception as e:
            rospy.logerr(f"Error executing speak: {e}")
            return False

    def execute_stop(self, parameters: Dict[str, Any]) -> bool:
        """
        Execute stop command
        """
        try:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            rospy.loginfo("Robot stopped")
            return True

        except Exception as e:
            rospy.logerr(f"Error executing stop: {e}")
            return False

# Main execution loop
def main():
    vta_system = AdvancedVoiceToActionSystem()

    rate = rospy.Rate(1)  # Process commands at 1 Hz

    while not rospy.is_shutdown():
        intent = vta_system.process_voice_command()
        if intent:
            rospy.loginfo(f"Processed intent: {intent}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

### Self-Hosted Whisper Example

For scenarios where privacy or latency is critical, you can use self-hosted Whisper:

```python
import whisper
import torch
import speech_recognition as sr
import os
from datetime import datetime

class SelfHostedVoiceToActionSystem:
    def __init__(self, model_size="small"):
        # Load Whisper model
        if torch.cuda.is_available():
            self.model = whisper.load_model(model_size).cuda()
        else:
            self.model = whisper.load_model(model_size)

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

    def transcribe_audio_file(self, audio_file_path: str) -> str:
        """
        Transcribe an audio file using self-hosted Whisper
        """
        result = self.model.transcribe(audio_file_path)
        return result["text"]

    def transcribe_microphone(self) -> str:
        """
        Transcribe audio from microphone using self-hosted Whisper
        """
        with self.microphone as source:
            print("Listening...")
            audio = self.recognizer.listen(source)

        # Save audio to temporary file
        with open("temp_audio.wav", "wb") as f:
            f.write(audio.get_wav_data())

        # Transcribe using Whisper
        result = self.model.transcribe("temp_audio.wav")

        # Clean up temporary file
        if os.path.exists("temp_audio.wav"):
            os.remove("temp_audio.wav")

        return result["text"]

# Example usage
if __name__ == "__main__":
    # Initialize the system with a smaller model for faster processing
    vta_system = SelfHostedVoiceToActionSystem(model_size="base")

    # Transcribe from microphone
    text = vta_system.transcribe_microphone()
    print(f"Transcribed: {text}")

    # Process the text for intent extraction
    # ... rest of voice-to-action pipeline
```

## Intent Extraction from Speech

Intent extraction is the process of identifying the underlying goal or action from a spoken command. This is a critical component of voice-to-action systems, as it bridges the gap between natural language understanding and robot action execution.

### Approaches to Intent Extraction

#### 1. Rule-Based Approaches
- **Pattern Matching**: Using predefined patterns to identify intents
- **Keyword Extraction**: Identifying key words that indicate specific actions
- **Template-Based**: Matching commands to predefined templates

**Advantages**:
- Fast and deterministic
- Easy to debug and modify
- Predictable behavior

**Disadvantages**:
- Limited flexibility
- Requires extensive manual pattern creation
- Struggles with complex or novel commands

#### 2. Machine Learning Approaches
- **Classification Models**: Training models to classify intents
- **Sequence Models**: Using RNNs or Transformers for sequence-to-sequence mapping
- **Pre-trained Language Models**: Leveraging models like GPT for understanding

**Advantages**:
- Better handling of natural language variations
- Can generalize to unseen commands
- More robust to variations in phrasing

**Disadvantages**:
- Requires training data
- Less interpretable
- Computational overhead

#### 3. Hybrid Approaches
- Combining rule-based and ML approaches
- Using rules for common commands, ML for complex ones
- Post-processing ML results with rules

### LLM-Based Intent Extraction

Large Language Models (LLMs) like GPT have revolutionized intent extraction by providing sophisticated natural language understanding capabilities:

```python
def extract_intent_with_llm(text_command):
    """
    Extract intent using a large language model
    """
    prompt = f"""
    You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'.

    The possible action types are:
    - NAVIGATE: For movement commands
    - GRASP: For object manipulation
    - SPEAK: For verbal responses
    - STOP: For stopping current actions
    - FOLLOW: For following a person or object
    - WAIT: For pausing or waiting

    Command: "{text_command}"

    Examples:
    - Command: "Go to the kitchen" → {{"action_type": "NAVIGATE", "parameters": {{"target_location": "kitchen"}}}}
    - Command: "Pick up the red cup" → {{"action_type": "GRASP", "parameters": {{"target_object": "red cup"}}}}
    - Command: "Say hello" → {{"action_type": "SPEAK", "parameters": {{"text": "hello"}}}}

    Respond with a JSON object containing 'action_type' and 'parameters'.
    """

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type' and 'parameters'."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.1
    )

    import json
    try:
        return json.loads(response.choices[0].message.content)
    except json.JSONDecodeError:
        # If JSON parsing fails, return a default action
        return {"action_type": "UNKNOWN", "parameters": {}}
```

### Context-Aware Intent Extraction

Advanced intent extraction systems consider context to improve accuracy:

```python
def extract_context_aware_intent(text_command, context):
    """
    Extract intent considering the current context
    """
    prompt = f"""
    You are a robot command interpreter. Extract the intent from the user's command considering the current context, and respond in JSON format with 'action_type' and 'parameters'.

    Current Context:
    - Robot Location: {context.get('robot_location', 'unknown')}
    - Available Objects: {context.get('available_objects', 'none')}
    - Current Task: {context.get('current_task', 'none')}
    - Time of Day: {context.get('time_of_day', 'unknown')}

    Command: "{text_command}"

    The possible action types are:
    - NAVIGATE: For movement commands
    - GRASP: For object manipulation
    - SPEAK: For verbal responses
    - STOP: For stopping current actions
    - FOLLOW: For following a person or object
    - WAIT: For pausing or waiting

    Consider the context when interpreting the command. For example, if the robot is in the kitchen and the user says "get me a drink", it might mean to get a glass of water from the refrigerator.

    Respond with a JSON object containing 'action_type' and 'parameters'.
    """

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a context-aware robot command interpreter. Extract the intent from the user's command considering the current context, and respond in JSON format with 'action_type' and 'parameters'."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.1
    )

    import json
    try:
        return json.loads(response.choices[0].message.content)
    except json.JSONDecodeError:
        return {"action_type": "UNKNOWN", "parameters": {}}
```

### Confidence Scoring and Validation

Intent extraction systems should provide confidence scores and validation:

```python
def extract_intent_with_confidence(text_command):
    """
    Extract intent with confidence scoring
    """
    prompt = f"""
    You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type', 'parameters', and 'confidence_score'.

    Command: "{text_command}"

    The possible action types are:
    - NAVIGATE: For movement commands
    - GRASP: For object manipulation
    - SPEAK: For verbal responses
    - STOP: For stopping current actions
    - FOLLOW: For following a person or object

    Provide a confidence score between 0.0 and 1.0 based on how certain you are about the interpretation.

    Respond with a JSON object containing 'action_type', 'parameters', and 'confidence_score'.
    """

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a robot command interpreter. Extract the intent from the user's command and respond in JSON format with 'action_type', 'parameters', and 'confidence_score'."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.1
    )

    import json
    try:
        result = json.loads(response.choices[0].message.content)

        # Validate confidence score
        if 'confidence_score' not in result:
            result['confidence_score'] = 0.8  # Default confidence

        return result
    except json.JSONDecodeError:
        return {
            "action_type": "UNKNOWN",
            "parameters": {},
            "confidence_score": 0.0
        }

def validate_intent(intent, robot_capabilities):
    """
    Validate that the extracted intent is achievable with robot capabilities
    """
    action_type = intent.get('action_type')

    if action_type == 'GRASP' and 'grasping' not in robot_capabilities:
        return False

    if action_type == 'NAVIGATE' and 'navigation' not in robot_capabilities:
        return False

    if action_type == 'SPEAK' and 'speech' not in robot_capabilities:
        return False

    return True
```

### Voice Command Categories

Voice commands for humanoid robots typically fall into several categories:

#### Navigation Commands
- "Go to the kitchen"
- "Move to the living room"
- "Approach the table"
- "Turn left/right"

#### Manipulation Commands
- "Pick up the cup"
- "Give me the book"
- "Put the object on the table"
- "Open the door"

#### Interaction Commands
- "Look at me"
- "Wave your hand"
- "Say hello"
- "Follow me"

#### Control Commands
- "Stop"
- "Pause"
- "Resume"
- "Return to base"

## Handling Ambiguity

Voice-to-action systems must handle ambiguous commands gracefully:

```python
def handle_ambiguous_command(intent):
    action_type = intent.get("action_type")
    parameters = intent.get("parameters", {})

    # Check for ambiguous parameters
    if action_type == "GRASP" and "target_object" in parameters:
        target = parameters["target_object"]
        if target.lower() == "it" or target.lower() == "that":
            # Ask for clarification
            return request_clarification(intent)

    return intent

def request_clarification(intent):
    # This would trigger a query to the user
    # For example: "Which object do you mean?"
    return {
        "action_type": "REQUEST_CLARIFICATION",
        "parameters": {"original_intent": intent}
    }
```

## Converting Speech to Robot-Understandable Intent

Converting natural speech to robot-understandable intent is a multi-step process that involves several transformations:

1. **Speech to Text**: Converting audio to textual representation
2. **Text to Intent**: Extracting the goal or action from the text
3. **Intent to Action**: Mapping the intent to specific robot commands
4. **Action to Execution**: Executing the commands on the robot

### Example Transformation Pipeline

Let's walk through an example of how the command "Please go to the kitchen and bring me a glass of water" gets transformed:

#### Step 1: Speech to Text
```
Audio Input: "Please go to the kitchen and bring me a glass of water"
Transcribed Text: "Please go to the kitchen and bring me a glass of water"
```

#### Step 2: Text to Intent
```
Input: "Please go to the kitchen and bring me a glass of water"
Processed Intent: {
  "action_type": "COMPLEX_TASK",
  "subtasks": [
    {
      "action_type": "NAVIGATE",
      "parameters": {"target_location": "kitchen"}
    },
    {
      "action_type": "GRASP",
      "parameters": {"target_object": "glass"}
    },
    {
      "action_type": "FILL",
      "parameters": {"container": "glass", "liquid": "water"}
    },
    {
      "action_type": "NAVIGATE",
      "parameters": {"target_location": "user_position"}
    },
    {
      "action_type": "PLACE",
      "parameters": {"target_location": "user_hand"}
    }
  ]
}
```

#### Step 3: Intent to Action
```
Processed Intent: {
  "action_type": "COMPLEX_TASK",
  "subtasks": [...]
}
Robot Actions: [
  "move_to_kitchen()",
  "find_glass()",
  "grasp_object('glass')",
  "fill_container('glass', 'water')",
  "move_to_user()",
  "offer_object('glass')"
]
```

### Detailed Example Implementation

Here's a complete example showing the transformation process:

```python
class SpeechToIntentConverter:
    def __init__(self):
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        openai.api_key = self.openai_api_key

    def convert_speech_to_intent(self, speech_text):
        """
        Complete pipeline: speech text → intent → robot actions
        """
        # Step 1: Extract high-level intent
        high_level_intent = self.extract_high_level_intent(speech_text)

        # Step 2: Break down complex intents into subtasks
        if high_level_intent['action_type'] == 'COMPLEX_TASK':
            subtasks = self.decompose_complex_task(high_level_intent)
            high_level_intent['subtasks'] = subtasks

        # Step 3: Map to robot-specific actions
        robot_actions = self.map_to_robot_actions(high_level_intent)

        return {
            "original_text": speech_text,
            "high_level_intent": high_level_intent,
            "robot_actions": robot_actions
        }

    def extract_high_level_intent(self, text_command):
        """
        Extract the main intent from the text command
        """
        prompt = f"""
        You are a robot command interpreter. Analyze the user's command and extract the main intent.

        Command: "{text_command}"

        Possible main action types:
        - SIMPLE_ACTION: Single action like "go to kitchen" or "pick up cup"
        - COMPLEX_TASK: Multiple-step task like "bring me water"
        - QUERY: Request for information
        - CONTROL: Stop, pause, resume commands

        Respond with a JSON object containing:
        - 'action_type': The main action type
        - 'primary_object': The main object involved (if any)
        - 'target_location': The target location (if any)
        - 'additional_info': Any other relevant information
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a robot command interpreter. Analyze the user's command and extract the main intent."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1
        )

        import json
        try:
            return json.loads(response.choices[0].message.content)
        except json.JSONDecodeError:
            return {
                "action_type": "UNKNOWN",
                "primary_object": None,
                "target_location": None,
                "additional_info": {}
            }

    def decompose_complex_task(self, intent):
        """
        Break down complex tasks into simpler subtasks
        """
        command = intent.get('additional_info', {}).get('original_command', '')

        prompt = f"""
        You are a task decomposition expert for robotics. Break down the following complex task into simpler, executable subtasks.

        Complex Task: "{command}"

        Possible subtask types:
        - NAVIGATE: Move to a location
        - DETECT: Identify objects in the environment
        - GRASP: Pick up an object
        - PLACE: Put down an object
        - FILL: Fill a container
        - EMPTY: Empty a container
        - SPEAK: Verbal response
        - WAIT: Pause execution

        Respond with a JSON array of subtasks, each containing 'action_type' and 'parameters'.
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a task decomposition expert for robotics. Break down complex tasks into simpler, executable subtasks."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1
        )

        import json
        try:
            return json.loads(response.choices[0].message.content)
        except json.JSONDecodeError:
            return []

    def map_to_robot_actions(self, intent):
        """
        Map the intent to specific robot action calls
        """
        if intent['action_type'] == 'COMPLEX_TASK' and 'subtasks' in intent:
            actions = []
            for subtask in intent['subtasks']:
                action = self.map_single_task_to_action(subtask)
                actions.append(action)
            return actions
        else:
            return [self.map_single_task_to_action(intent)]

    def map_single_task_to_action(self, task):
        """
        Map a single task to a robot action call
        """
        action_type = task.get('action_type', 'UNKNOWN')

        if action_type == 'NAVIGATE':
            target = task.get('parameters', {}).get('target_location', 'unknown')
            return f"move_to_location('{target}')"
        elif action_type == 'GRASP':
            obj = task.get('parameters', {}).get('target_object', 'unknown')
            return f"grasp_object('{obj}')"
        elif action_type == 'PLACE':
            location = task.get('parameters', {}).get('target_location', 'unknown')
            return f"place_object_at('{location}')"
        elif action_type == 'SPEAK':
            text = task.get('parameters', {}).get('text', 'Hello')
            return f"say('{text}')"
        else:
            return f"execute_unknown_task('{action_type}')"

# Example usage
converter = SpeechToIntentConverter()
result = converter.convert_speech_to_intent("Please go to the kitchen and bring me a glass of water")
print(f"Original: {result['original_text']}")
print(f"Intent: {result['high_level_intent']}")
print(f"Actions: {result['robot_actions']}")
```

### Real-World Examples

#### Example 1: Simple Navigation Command
```
Input: "Go to the living room"
Output Intent: {
  "action_type": "NAVIGATE",
  "parameters": {
    "target_location": "living room",
    "confidence": 0.95
  }
}
Robot Action: move_to_location("living room")
```

#### Example 2: Object Manipulation Command
```
Input: "Pick up the red book from the table"
Output Intent: {
  "action_type": "GRASP",
  "parameters": {
    "target_object": {
      "color": "red",
      "type": "book",
      "location": "table"
    },
    "confidence": 0.89
  }
}
Robot Action: find_and_grasp_object(color="red", type="book", location="table")
```

#### Example 3: Complex Multi-Step Command
```
Input: "Could you please go to the kitchen, find a clean glass, fill it with water, and bring it to me?"
Output Intent: {
  "action_type": "COMPLEX_TASK",
  "subtasks": [
    {
      "action_type": "NAVIGATE",
      "parameters": {"target_location": "kitchen"},
      "confidence": 0.95
    },
    {
      "action_type": "DETECT",
      "parameters": {"object_type": "glass", "condition": "clean"},
      "confidence": 0.87
    },
    {
      "action_type": "GRASP",
      "parameters": {"target_object_id": "glass_001"},
      "confidence": 0.92
    },
    {
      "action_type": "FILL",
      "parameters": {"container_id": "glass_001", "liquid": "water"},
      "confidence": 0.90
    },
    {
      "action_type": "NAVIGATE",
      "parameters": {"target_location": "user_position"},
      "confidence": 0.95
    },
    {
      "action_type": "PLACE",
      "parameters": {"target_location": "user_hand"},
      "confidence": 0.88
    }
  ],
  "confidence": 0.91
}
Robot Actions: [
  "move_to_location('kitchen')",
  "find_object(type='glass', condition='clean')",
  "grasp_object('glass_001')",
  "fill_container('glass_001', 'water')",
  "move_to_location('user_position')",
  "offer_object('glass_001')"
]
```

## Error Handling and Robustness

Voice-to-action systems must handle various error conditions:

### Audio Quality Issues
- Low volume or noisy environments
- Audio clipping or distortion
- Background noise interference

### Recognition Errors
- Misrecognized words or phrases
- Accents or speech patterns not well-handled by ASR
- Technical jargon or domain-specific terminology

### Intent Ambiguity
- Commands that could have multiple interpretations
- Commands that conflict with robot capabilities
- Commands that are unsafe or infeasible

## Performance Considerations

### Latency
- Minimize the time between speech and action execution
- Optimize network calls to Whisper API
- Consider local Whisper models for reduced latency

### Accuracy
- Balance between false positives and false negatives
- Implement confidence thresholds for command execution
- Use context to improve accuracy

### Privacy
- Consider data privacy when using cloud-based ASR
- Implement local processing where sensitive information is involved
- Follow appropriate data handling practices

## Integration with ROS 2

Voice-to-action systems can be integrated with ROS 2 using action servers and clients:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class VoiceCommandActionClient(Node):
    def __init__(self):
        super().__init__('voice_command_client')
        self._action_client = ActionClient(
            self,
            VoiceCommand,  # Custom action message
            'voice_command'
        )

    def send_voice_command(self, text, intent):
        goal_msg = VoiceCommand.Goal()
        goal_msg.command_text = text
        goal_msg.intent = intent

        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
```

## Exercises for Students

1. **Basic Implementation**: Implement a simple voice-to-action system that can recognize and execute the commands "move forward", "turn left", and "stop". Test the system with different audio inputs and evaluate its accuracy.

2. **Intent Extraction Challenge**: Modify the intent extraction example to handle commands with multiple objects (e.g., "pick up the red cup and the blue book"). How would you extend the system to handle these complex requests?

3. **Context Integration**: Add context awareness to the intent extraction system. For example, if the robot knows it's in the kitchen, how would this influence the interpretation of commands like "get me something to drink"?

4. **Error Handling**: Implement error handling for cases where the Whisper transcription confidence is low. What strategies would you use to handle uncertain transcriptions?

5. **Multi-Step Commands**: Extend the system to handle multi-step commands like "go to the kitchen, find a glass, and bring it to the living room". How would you break down and execute these complex tasks?

6. **Privacy Considerations**: Design a version of the voice-to-action system that processes audio locally without sending data to cloud APIs. What trade-offs would you need to consider?

7. **Performance Optimization**: Implement a caching mechanism for common commands to reduce API calls to Whisper and the LLM. How would you determine which commands to cache?

8. **ROS 2 Integration**: Create a ROS 2 package that implements the voice-to-action system as a service. How would you structure the nodes and message types?

## API Contract Examples for Voice Processing Services

In production voice-to-action systems, it's important to define clear API contracts for the various services involved. Here are examples of API contracts for voice processing services:

### Whisper ASR Service API Contract

#### Endpoint: `/api/v1/asr/transcribe`

**Method**: POST

**Request**:
```json
{
  "audio_data": "base64_encoded_audio",
  "language": "en",
  "model": "whisper-1",
  "format": "mp3|wav|flac|...",
  "timeout": 30
}
```

**Response**:
```json
{
  "transcription": "text of the spoken command",
  "confidence": 0.95,
  "language": "en",
  "processing_time": 1.2,
  "word_timestamps": [
    {"word": "hello", "start": 0.0, "end": 0.5},
    {"word": "world", "start": 0.6, "end": 1.0}
  ]
}
```

**Error Response**:
```json
{
  "error": "transcription_failed",
  "message": "Audio data could not be processed",
  "code": "ASR_001"
}
```

### Intent Extraction Service API Contract

#### Endpoint: `/api/v1/nlp/extract-intent`

**Method**: POST

**Request**:
```json
{
  "text": "Please navigate to the kitchen",
  "context": {
    "robot_state": "idle",
    "environment": "home",
    "timestamp": "2024-01-01T10:00:00Z"
  },
  "robot_capabilities": ["navigate", "grasp", "speak"]
}
```

**Response**:
```json
{
  "intent": {
    "action_type": "NAVIGATE",
    "parameters": {
      "target_location": "kitchen"
    },
    "confidence": 0.89,
    "confidence_score": 0.89
  },
  "processing_time": 0.5
}
```

### Voice Command Processing Service API Contract

#### Endpoint: `/api/v1/vla/process-voice-command`

**Method**: POST

**Request**:
```json
{
  "audio_data": "base64_encoded_audio",
  "language": "en",
  "context": {
    "robot_state": "idle",
    "environment_map": "current_environment_map",
    "available_objects": ["cup", "book", "phone"],
    "robot_location": "living_room"
  }
}
```

**Response**:
```json
{
  "status": "success",
  "transcription": "Go to the kitchen and bring me a glass of water",
  "intent": {
    "action_type": "COMPLEX_TASK",
    "subtasks": [
      {
        "action_type": "NAVIGATE",
        "parameters": {"target_location": "kitchen"},
        "confidence": 0.95
      },
      {
        "action_type": "GRASP",
        "parameters": {"target_object": "glass"},
        "confidence": 0.89
      }
    ]
  },
  "execution_id": "exec_12345",
  "estimated_completion": 120.0
}
```

### Service Integration Example

Here's how these services would work together in a complete voice-to-action pipeline:

```python
class VoiceToActionService:
    def __init__(self, asr_url, nlp_url, robot_control_url):
        self.asr_url = asr_url
        self.nlp_url = nlp_url
        self.robot_control_url = robot_control_url

    def process_voice_command(self, audio_data, context):
        """
        Complete voice command processing pipeline
        """
        # Step 1: Transcribe audio
        transcription_response = requests.post(
            f"{self.asr_url}/api/v1/asr/transcribe",
            json={
                "audio_data": audio_data,
                "language": "en",
                "model": "whisper-1"
            }
        )

        if transcription_response.status_code != 200:
            raise Exception("ASR service failed")

        transcription = transcription_response.json()["transcription"]

        # Step 2: Extract intent
        intent_response = requests.post(
            f"{self.nlp_url}/api/v1/nlp/extract-intent",
            json={
                "text": transcription,
                "context": context,
                "robot_capabilities": ["navigate", "grasp", "speak"]
            }
        )

        if intent_response.status_code != 200:
            raise Exception("NLP service failed")

        intent = intent_response.json()["intent"]

        # Step 3: Execute robot action
        execution_response = requests.post(
            f"{self.robot_control_url}/api/v1/robot/execute",
            json={
                "intent": intent,
                "context": context
            }
        )

        if execution_response.status_code != 200:
            raise Exception("Robot execution failed")

        return execution_response.json()

# Example usage
vta_service = VoiceToActionService(
    asr_url="http://asr-service:8000",
    nlp_url="http://nlp-service:8000",
    robot_control_url="http://robot-control:8000"
)

result = vta_service.process_voice_command(audio_data, context)
```

## Summary

Voice-to-action systems using OpenAI Whisper provide a natural and intuitive way for humans to interact with humanoid robots. The complete pipeline includes audio capture, speech recognition, intent extraction, and command mapping to robot actions.

Key considerations for implementation include:
- Handling ambiguity and error conditions gracefully
- Optimizing for latency and accuracy
- Ensuring privacy and security
- Integrating effectively with robot control systems

The next chapter will explore cognitive planning with large language models, which builds on these voice-to-action concepts to enable more sophisticated task planning and execution.
# API Contracts: Vision-Language-Action (VLA) System Interfaces

**Feature**: Vision-Language-Action (VLA) for Humanoid Robotics
**Created**: 2025-12-24
**Spec**: [specs/4-vla-education/spec.md](specs/4-vla-education/spec.md)

## Overview

This document defines the API contracts and interfaces that students will learn about in the Vision-Language-Action (VLA) educational module. These contracts represent the key interfaces between different components of a VLA system.

## Voice-to-Action Service Contract

### Whisper ASR Service Interface

**Endpoint**: `/api/v1/asr/transcribe`

**Method**: POST

**Request**:
```json
{
  "audio_data": "base64_encoded_audio",
  "language": "en",
  "model": "whisper-1"
}
```

**Response**:
```json
{
  "transcription": "text of the spoken command",
  "confidence": 0.95,
  "language": "en",
  "processing_time": 1.2
}
```

**Error Response**:
```json
{
  "error": "transcription_failed",
  "message": "Audio data could not be processed"
}
```

### Intent Extraction Service Interface

**Endpoint**: `/api/v1/nlp/extract-intent`

**Method**: POST

**Request**:
```json
{
  "text": "Please navigate to the kitchen",
  "context": {
    "robot_state": "idle",
    "environment": "home"
  }
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
    "confidence": 0.89
  }
}
```

## Vision Processing Service Contract

### Object Detection Interface

**Endpoint**: `/api/v1/vision/detect-objects`

**Method**: POST

**Request**:
```json
{
  "image_data": "base64_encoded_image",
  "model": "yolo-v8"
}
```

**Response**:
```json
{
  "objects": [
    {
      "label": "cup",
      "confidence": 0.92,
      "bbox": {
        "x_min": 100,
        "y_min": 150,
        "x_max": 200,
        "y_max": 250
      },
      "position_3d": {
        "x": 1.2,
        "y": 0.5,
        "z": 0.8
      }
    }
  ],
  "processing_time": 0.08
}
```

## Cognitive Planning Service Contract

### Natural Language to Action Sequence Interface

**Endpoint**: `/api/v1/planning/generate-sequence`

**Method**: POST

**Request**:
```json
{
  "goal": "Pick up the red cup from the table and place it in the sink",
  "robot_capabilities": ["grasp", "navigate", "manipulate"],
  "environment_context": {
    "objects": ["red cup", "table", "sink"],
    "locations": ["kitchen", "dining area"]
  }
}
```

**Response**:
```json
{
  "action_sequence": {
    "id": "seq_001",
    "name": "FetchAndPlaceCup",
    "steps": [
      {
        "step_number": 1,
        "action_type": "NAVIGATE",
        "parameters": {
          "target_location": "table"
        }
      },
      {
        "step_number": 2,
        "action_type": "DETECT_OBJECTS",
        "parameters": {
          "target_object": "red cup"
        }
      },
      {
        "step_number": 3,
        "action_type": "GRASP",
        "parameters": {
          "object_id": "red cup"
        }
      },
      {
        "step_number": 4,
        "action_type": "NAVIGATE",
        "parameters": {
          "target_location": "sink"
        }
      },
      {
        "step_number": 5,
        "action_type": "RELEASE",
        "parameters": {}
      }
    ],
    "estimated_duration": 120.0,
    "confidence": 0.85
  }
}
```

## ROS 2 Action Contracts

### Navigation Action Interface

**Action Type**: `nav2_msgs/action/NavigateToPose`

**Goal**:
```json
{
  "pose": {
    "position": {
      "x": 1.0,
      "y": 2.0,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  }
}
```

**Result**:
```json
{
  "result_code": 1,
  "message": "Goal reached"
}
```

### Manipulation Action Interface

**Action Type**: `custom_manipulation_msgs/action/GraspObject`

**Goal**:
```json
{
  "object_id": "red_cup_001",
  "grasp_type": "top_grasp",
  "force_limit": 10.0
}
```

**Result**:
```json
{
  "success": true,
  "grasp_quality": 0.95,
  "message": "Object grasped successfully"
}
```

## VLA System Integration Contract

### Unified VLA Interface

**Endpoint**: `/api/v1/vla/process-command`

**Method**: POST

**Request**:
```json
{
  "input_type": "voice", // or "text"
  "input_data": "base64_encoded_audio_or_text",
  "context": {
    "vision_data": "latest_image_data",
    "robot_state": "current_robot_state",
    "environment_map": "current_environment_map"
  }
}
```

**Response**:
```json
{
  "status": "processing",
  "action_sequence": {
    "id": "vla_seq_001",
    "steps": [
      // Action sequence steps as defined above
    ]
  },
  "execution_id": "exec_001",
  "estimated_completion": 150.0
}
```

## Error Handling and Status Codes

### Standard Error Response Format
```json
{
  "error_code": "string",
  "message": "human-readable error message",
  "details": {
    // optional error-specific details
  },
  "timestamp": "ISO 8601 timestamp"
}
```

### Common Error Codes
- `INPUT_INVALID`: Input data is malformed or invalid
- `PROCESSING_FAILED`: Service could not process the request
- `RESOURCE_UNAVAILABLE`: Required resource is not available
- `PERMISSION_DENIED`: Insufficient permissions to access resource
- `TIMEOUT`: Request timed out before completion
- `QUOTA_EXCEEDED`: API quota has been exceeded

## Authentication and Security

### API Key Authentication
All VLA service endpoints require an API key in the header:
```
Authorization: Bearer YOUR_API_KEY
```

### Rate Limiting
- Whisper ASR: 100 requests per minute
- Intent Extraction: 200 requests per minute
- Vision Processing: 50 requests per minute
- Cognitive Planning: 20 requests per minute
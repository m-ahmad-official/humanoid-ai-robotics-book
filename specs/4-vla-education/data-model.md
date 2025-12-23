# Data Model: Vision-Language-Action (VLA) for Humanoid Robotics

**Feature**: Vision-Language-Action (VLA) for Humanoid Robotics
**Created**: 2025-12-24
**Spec**: [specs/4-vla-education/spec.md](specs/4-vla-education/spec.md)

## Overview

This document defines the key data models and entities for the Vision-Language-Action (VLA) educational module. These models represent the core concepts students will learn about and implement as part of their understanding of VLA systems in humanoid robotics.

## Core Entities

### 1. VisionData
Represents visual information captured by the robot's sensors

```
VisionData {
  id: string
  timestamp: datetime
  image_data: binary | string (URL/path)
  depth_data: binary | float[] (depth map)
  camera_info: CameraInfo
  objects: ObjectDetection[]
  embeddings: float[] (CLIP features)
  processed_by: string (model name)
}
```

**Relationships:**
- One-to-Many: VisionData → ObjectDetection
- Many-to-One: VisionData ← RobotState (current vision input)

### 2. ObjectDetection
Represents detected objects in the visual field

```
ObjectDetection {
  id: string
  vision_data_id: string
  label: string
  confidence: float (0.0 - 1.0)
  bounding_box: BoundingBox
  segmentation_mask: binary | null
  position_3d: Point3D
}
```

**Relationships:**
- Many-to-One: ObjectDetection → VisionData
- Many-to-Many: ObjectDetection ↔ SemanticConcept

### 3. BoundingBox
Defines the 2D boundary of a detected object

```
BoundingBox {
  x_min: float
  y_min: float
  x_max: float
  y_max: float
  width: float
  height: float
}
```

### 4. Point3D
Represents a 3D coordinate in space

```
Point3D {
  x: float
  y: float
  z: float
}
```

### 5. NaturalLanguageIntent
Represents the extracted meaning from human speech or text commands

```
NaturalLanguageIntent {
  id: string
  original_input: string
  processed_text: string
  intent_type: IntentType
  parameters: map<string, string>
  confidence: float (0.0 - 1.0)
  extracted_at: datetime
  source: InputSource
}
```

**Relationships:**
- One-to-Many: NaturalLanguageIntent → ActionSequence
- Many-to-One: NaturalLanguageIntent ← VoiceCommand

### 6. IntentType (Enum)
Defines the types of intents that can be extracted

```
IntentType {
  NAVIGATE_TO: "Navigate to a location"
  GRASP_OBJECT: "Grasp a specific object"
  PERFORM_ACTION: "Perform a specific action"
  ANSWER_QUESTION: "Answer a question about the environment"
  FOLLOW_PATH: "Follow a specific path"
  AVOID_OBSTACLE: "Avoid obstacles in path"
  STOP_ACTION: "Stop current action"
}
```

### 7. VoiceCommand
Represents voice input processed by ASR systems

```
VoiceCommand {
  id: string
  audio_data: binary
  transcription: string
  confidence: float (0.0 - 1.0)
  language: string
  processed_by: string (ASR model/API)
  intent: NaturalLanguageIntent
  timestamp: datetime
}
```

**Relationships:**
- One-to-One: VoiceCommand → NaturalLanguageIntent

### 8. ActionSequence
Represents a series of actions to be executed by the robot

```
ActionSequence {
  id: string
  name: string
  description: string
  steps: ActionStep[]
  priority: Priority
  created_by: string (LLM model)
  created_at: datetime
  estimated_duration: float (seconds)
  dependencies: string[] (other action sequence IDs)
}
```

**Relationships:**
- One-to-Many: ActionSequence → ActionStep
- Many-to-Many: ActionSequence ↔ NaturalLanguageIntent

### 9. ActionStep
Represents a single action within an action sequence

```
ActionStep {
  id: string
  action_sequence_id: string
  step_number: int
  action_type: ActionType
  parameters: map<string, any>
  preconditions: string[] (conditions that must be true)
  postconditions: string[] (conditions that will be true after execution)
  timeout: float (seconds)
  retry_count: int
}
```

### 10. ActionType (Enum)
Defines the types of actions that can be executed

```
ActionType {
  MOVE_TO: "Move to a specific location"
  GRASP: "Grasp an object"
  RELEASE: "Release a grasped object"
  ROTATE: "Rotate the robot/base"
  SPEAK: "Speak a response"
  LISTEN: "Listen for voice commands"
  LOOK_AT: "Look at a specific point"
  DETECT_OBJECTS: "Detect objects in view"
  NAVIGATE: "Navigate to a target"
  WAIT: "Wait for a condition or duration"
  EXECUTE_SUBSEQUENCE: "Execute another action sequence"
}
```

### 11. RobotState
Represents the current state of the humanoid robot

```
RobotState {
  id: string
  timestamp: datetime
  position: Point3D
  orientation: Quaternion
  battery_level: float (0.0 - 1.0)
  joint_states: map<string, JointState>
  active_actions: string[] (currently executing action IDs)
  vision_data: VisionData
  last_intent: NaturalLanguageIntent
  status: RobotStatus
}
```

### 12. JointState
Represents the state of a single robot joint

```
JointState {
  position: float
  velocity: float
  effort: float
}
```

### 13. Quaternion
Represents 3D rotation

```
Quaternion {
  x: float
  y: float
  z: float
  w: float
}
```

### 14. RobotStatus (Enum)
Current operational status of the robot

```
RobotStatus {
  IDLE: "Robot is idle"
  LISTENING: "Robot is listening for commands"
  PROCESSING: "Processing a command"
  EXECUTING: "Executing an action sequence"
  ERROR: "Robot is in error state"
  CHARGING: "Robot is charging"
  SAFETY_LOCK: "Robot safety lock engaged"
}
```

### 15. CognitivePlan
Represents the high-level plan generated by LLM for complex tasks

```
CognitivePlan {
  id: string
  goal: string (natural language goal)
  plan_description: string
  action_sequences: ActionSequence[]
  dependencies: string[] (plan IDs this plan depends on)
  estimated_completion: float (seconds)
  confidence: float (0.0 - 1.0)
  generated_by: string (LLM model)
  generated_at: datetime
  execution_history: PlanExecution[]
}
```

**Relationships:**
- One-to-Many: CognitivePlan → ActionSequence
- One-to-Many: CognitivePlan → PlanExecution

### 16. PlanExecution
Represents an execution attempt of a cognitive plan

```
PlanExecution {
  id: string
  cognitive_plan_id: string
  start_time: datetime
  end_time: datetime | null
  status: ExecutionStatus
  success_rate: float (0.0 - 1.0)
  error_log: string | null
  executed_steps: PlanStepExecution[]
}
```

### 17. PlanStepExecution
Represents the execution of a single step in a plan

```
PlanStepExecution {
  id: string
  plan_execution_id: string
  step_number: int
  action_step_id: string
  start_time: datetime
  end_time: datetime | null
  status: ExecutionStatus
  result: string | null
  error_message: string | null
}
```

### 18. ExecutionStatus (Enum)
Status of plan or action execution

```
ExecutionStatus {
  PENDING: "Not yet started"
  RUNNING: "Currently executing"
  SUCCESS: "Completed successfully"
  FAILED: "Execution failed"
  CANCELLED: "Cancelled by user or system"
  TIMEOUT: "Execution timed out"
}
```

### 19. InputSource (Enum)
Source of input to the VLA system

```
InputSource {
  VOICE: "Voice command via microphone"
  TEXT: "Text input directly"
  GESTURE: "Gesture recognition input"
  ENVIRONMENTAL: "Environmental sensor input"
  REMOTE: "Remote command from external system"
}
```

### 20. Priority (Enum)
Priority level for action sequences

```
Priority {
  CRITICAL: 5
  HIGH: 4
  MEDIUM: 3
  LOW: 2
  BACKGROUND: 1
}
```

## Data Flow Relationships

### VLA Integration Flow
```
VoiceCommand -(processed to)-> NaturalLanguageIntent -(converted to)-> ActionSequence -(executed as)-> RobotState
VisionData -(processed with)-> ObjectDetection -(influences)-> ActionSequence -(affects)-> RobotState
NaturalLanguageIntent -(combined with)-> VisionData -(generates)-> CognitivePlan
```

### Educational Data Model Usage
These data models will be used in educational examples to demonstrate:
1. How visual information is structured and processed
2. How voice commands are converted to actionable intents
3. How high-level goals are decomposed into action sequences
4. How robot state changes during execution
5. How cognitive planning integrates multiple modalities

## Validation Rules

### Business Rules
1. Each NaturalLanguageIntent must have a confidence score above 0.7 to be processed
2. Action sequences must have valid preconditions before execution
3. RobotState updates must occur within 100ms of sensor input
4. Cognitive plans must be decomposable into executable action sequences

### Constraints
1. Maximum 100 objects can be detected in a single VisionData instance
2. ActionSequence cannot exceed 50 steps
3. PlanExecution timeout defaults to 300 seconds
4. VoiceCommand transcription confidence must be >0.8 for processing

## Serialization Formats

### JSON Schema Examples

**VisionData Example:**
```json
{
  "id": "vision_001",
  "timestamp": "2025-12-24T10:30:00Z",
  "image_data": "data:image/jpeg;base64,...",
  "objects": [
    {
      "id": "obj_001",
      "label": "cup",
      "confidence": 0.95,
      "bounding_box": {
        "x_min": 100,
        "y_min": 150,
        "x_max": 200,
        "y_max": 250
      }
    }
  ]
}
```

**ActionSequence Example:**
```json
{
  "id": "seq_001",
  "name": "FetchCoffee",
  "steps": [
    {
      "id": "step_001",
      "action_type": "NAVIGATE",
      "parameters": {
        "target_location": "kitchen"
      }
    },
    {
      "id": "step_002",
      "action_type": "GRASP",
      "parameters": {
        "object_id": "cup"
      }
    }
  ]
}
```
# Data Model: ROS 2 for Physical AI Education

## Key Entities

### ROS 2 Node
- **Name**: ROS 2 Node
- **Description**: A process that performs computation in the ROS 2 system
- **Fields**:
  - node_name: string (unique identifier for the node)
  - node_namespace: string (optional namespace for the node)
  - publishers: list of Publisher objects
  - subscribers: list of Subscriber objects
  - services: list of Service objects
  - clients: list of Client objects
- **Relationships**: Can publish to Topics, subscribe to Topics, provide Services, call Clients
- **Validation**: Node name must be unique within namespace

### Topic
- **Name**: Topic
- **Description**: Named bus over which nodes exchange messages
- **Fields**:
  - topic_name: string (unique identifier for the topic)
  - message_type: string (type of messages published/subscribed)
  - publishers_count: integer (number of publishers)
  - subscribers_count: integer (number of subscribers)
- **Relationships**: Connected to multiple Nodes via Publisher/Subscriber
- **Validation**: Topic names must follow ROS naming conventions

### Message
- **Name**: Message
- **Description**: Data structure used for communication between nodes
- **Fields**:
  - message_type: string (type definition, e.g., std_msgs/String)
  - data: object (actual message content)
  - timestamp: datetime (when message was created)
- **Relationships**: Used by Publishers to send, Subscribers to receive
- **Validation**: Must match the defined message type schema

### Service
- **Name**: Service
- **Description**: Synchronous request-response communication pattern
- **Fields**:
  - service_name: string (unique identifier for the service)
  - service_type: string (type definition for request/response)
  - server: Node (the node providing the service)
  - clients: list of Node (nodes calling the service)
- **Relationships**: Connected to one Server Node and multiple Client Nodes
- **Validation**: Service name must be unique within namespace

### URDF Model
- **Name**: URDF Model
- **Description**: XML-based robot description format
- **Fields**:
  - robot_name: string (name of the robot)
  - links: list of Link objects (rigid components)
  - joints: list of Joint objects (connections between links)
  - materials: list of Material objects (visual properties)
- **Relationships**: Links connected via Joints to form kinematic chains
- **Validation**: Must follow URDF XML schema and form valid kinematic structure

### Link
- **Name**: Link
- **Description**: Rigid component of a robot in URDF
- **Fields**:
  - link_name: string (unique identifier for the link)
  - visual: object (visual representation)
  - collision: object (collision properties)
  - inertial: object (mass and inertia properties)
- **Relationships**: Connected to other Links via Joint objects
- **Validation**: Must have valid physical properties

### Joint
- **Name**: Joint
- **Description**: Connection between two links in URDF
- **Fields**:
  - joint_name: string (unique identifier for the joint)
  - joint_type: string (revolute, prismatic, fixed, etc.)
  - parent_link: string (name of parent link)
  - child_link: string (name of child link)
  - limits: object (movement constraints)
- **Relationships**: Connects parent Link to child Link
- **Validation**: Must connect valid parent and child links

### rclpy Components
- **Name**: rclpy Components
- **Description**: Python client library for ROS 2
- **Fields**:
  - node_interface: object (core node functionality)
  - publisher: object (publishing capabilities)
  - subscriber: object (subscription capabilities)
  - service: object (service server capabilities)
  - client: object (service client capabilities)
- **Relationships**: Used by Python programs to interface with ROS 2
- **Validation**: Must follow rclpy API specifications

## State Transitions

### Node Lifecycle
- **States**: Uninitialized → Initialized → Active → Inactive → Shutdown
- **Transitions**:
  - Created → Initialized: When node is created and initialized
  - Initialized → Active: When node is started
  - Active ↔ Inactive: When node is paused/resumed
  - Active/Inactive → Shutdown: When node is destroyed

### Communication Patterns
- **Publish-Subscribe**: Publisher sends messages → Topic routes → Subscribers receive
- **Service-Client**: Client sends request → Service processes → Client receives response
- **Action**: Goal sent → Feedback provided → Result returned

## Validation Rules

1. **ROS 2 Communication**: All nodes must follow ROS 2 naming conventions
2. **URDF Validity**: URDF models must form valid kinematic chains without loops
3. **Message Compatibility**: Publishers and subscribers must use compatible message types
4. **Service Contracts**: Service requests and responses must match defined interfaces
5. **Namespace Management**: Names must be unique within their namespace scope
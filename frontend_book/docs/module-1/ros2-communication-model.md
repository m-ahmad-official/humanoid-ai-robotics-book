---
sidebar_position: 2
title: ROS 2 Communication Model
---

# ROS 2 Communication Model

## Nodes

In ROS 2, a **Node** is the fundamental unit of execution. Nodes are processes that perform computation and communicate with other nodes through messages. Each node typically performs a specific task or function within the larger robotic system.

### Node Characteristics

- **Process-based**: Each node runs as a separate process
- **Single-threaded execution**: By default, nodes execute callbacks in a single thread
- **Namespaced**: Nodes can have namespaces for organization
- **Composable**: Multiple nodes can be combined into a single process for efficiency

### Creating Nodes

Nodes are created by inheriting from the `Node` class in your chosen client library (rclpy for Python, rclcpp for C++):

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize node-specific components here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Publish/Subscribe Data Flow

**Topics** are named buses over which nodes exchange messages. The publish/subscribe pattern is the most common communication paradigm in ROS 2.

### Publishers and Subscribers

- **Publishers**: Send messages to topics
- **Subscribers**: Receive messages from topics
- **Anonymous**: Publishers and subscribers don't know about each other
- **Decoupled**: Publishers and subscribers can start and stop at any time

### Example: Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

**Services** provide a request/reply communication pattern where one node sends a request and another node provides a response.

### Service Characteristics

- **Synchronous**: The client waits for a response
- **Request/Response**: Defined message types for request and response
- **One-to-one**: Typically one server responds to one client at a time

### Example: Service Server

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Service Client

```python
import sys
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridging Python AI Agents to Robot Controllers using rclpy

One of the most powerful aspects of ROS 2 is its ability to bridge AI agents written in Python with robot controllers. This allows sophisticated AI algorithms to control physical robots through standardized interfaces.

### rclpy Overview

`rclpy` is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl) and the underlying middleware.

### Integration Pattern

The typical pattern for bridging AI agents to robot controllers involves:

1. **Perception Nodes**: Process sensor data and publish to topics
2. **AI Decision Node**: Subscribes to sensor data, runs AI algorithms, publishes commands
3. **Controller Nodes**: Subscribe to commands and control robot actuators

### Example: AI Agent Bridge

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class AIAgentBridge(Node):
    def __init__(self):
        super().__init__('ai_agent_bridge')

        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)

        # Publish commands to robot
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        self.laser_data = None

    def laser_callback(self, msg):
        # Store laser scan data for AI processing
        self.laser_data = msg.ranges

    def ai_decision_callback(self):
        if self.laser_data is None:
            return

        # Simple AI decision making
        # Find minimum distance in front of robot
        front_distances = self.laser_data[0:30] + self.laser_data[-30:]
        min_distance = min(front_distances)

        # Create velocity command based on AI decision
        cmd_vel = Twist()

        if min_distance < 1.0:  # Too close to obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn away
        else:
            cmd_vel.linear.x = 0.5  # Move forward
            cmd_vel.angular.z = 0.0

        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentBridge()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## The Perception → Decision → Action Loop

The perception-decision-action loop is a fundamental concept in robotics and AI that describes how intelligent agents interact with their environment.

### Loop Components

1. **Perception**: Gathering information from sensors about the environment
2. **Decision**: Processing sensor data to determine appropriate actions
3. **Action**: Executing commands that affect the environment
4. **Feedback**: New sensor data reflecting the results of actions

### ROS 2 Implementation

In ROS 2, this loop is typically implemented using:

- **Perception**: Sensor drivers publishing to topics
- **Decision**: AI nodes subscribing to sensor data and publishing commands
- **Action**: Controller nodes executing commands on the robot
- **Feedback**: Sensor data reflecting the results of actions

### Example Complete Loop

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionDecisionActionLoop(Node):
    def __init__(self):
        super().__init__('pda_loop')

        # Initialize components
        self.cv_bridge = CvBridge()

        # Perception: Subscribe to sensor data
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)

        # Action: Publish commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.decision_callback)

        # Internal state
        self.latest_image = None
        self.latest_laser = None

    def image_callback(self, msg):
        """Perception: Process camera image"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')

    def laser_callback(self, msg):
        """Perception: Process laser scan"""
        self.latest_laser = msg.ranges

    def decision_callback(self):
        """Decision: Make decisions based on sensor data"""
        if self.latest_laser is None:
            return

        # Simple decision logic
        cmd_vel = Twist()

        # Analyze laser data for obstacles
        front_ranges = self.latest_laser[0:30] + self.latest_laser[-30:]
        min_front = min([r for r in front_ranges if r > 0.1], default=float('inf'))

        # Decision making
        if min_front < 0.8:  # Obstacle detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.8  # Turn right
        else:
            cmd_vel.linear.x = 0.4  # Move forward
            cmd_vel.angular.z = 0.0

        # Action: Execute command
        self.cmd_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    loop_node = PerceptionDecisionActionLoop()
    rclpy.spin(loop_node)
    loop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

The ROS 2 communication model provides a flexible and robust framework for building complex robotic systems. The combination of nodes, topics, and services enables the creation of distributed systems where AI agents can seamlessly interface with robot controllers. The perception-decision-action loop provides a fundamental pattern for implementing intelligent behavior in robotic systems.
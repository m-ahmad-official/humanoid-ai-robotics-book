# Quickstart Guide: ROS 2 for Physical AI Education

## Prerequisites

- Node.js 18+ and npm/yarn
- Python 3.8+ with pip
- ROS 2 Humble Hawksbill installed (for running examples)
- Basic Python knowledge

## Getting Started

### 1. Clone and Setup the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Docusaurus Dependencies

```bash
npm install
```

### 3. Install ROS 2 Dependencies (for examples)

```bash
# Follow ROS 2 Humble installation guide for your OS
# Install rclpy: sudo apt install python3-ros-environment python3-ros-workspace python3-roslib python3-rosdep python3-vcstool
```

### 4. Start the Development Server

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### 5. Navigate the Educational Content

The educational content for Module 1 is organized as follows:

- **Introduction to ROS 2 for Physical AI**: Foundational concepts about ROS 2 and its role in embodied intelligence
- **ROS 2 Communication Model**: Understanding nodes, topics, services, and the perception → decision → action loop
- **Robot Structure with URDF**: Learning about URDF, links, joints, and kinematic chains for humanoid robots

## Running Code Examples

Each chapter includes practical code examples that demonstrate the concepts discussed:

1. Navigate to the specific chapter in the documentation
2. Find the code example section
3. The examples are written in Python using rclpy
4. You can run the examples in your ROS 2 environment

## Building for Production

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

The site is configured for deployment to GitHub Pages. After building, the content can be deployed using GitHub Actions or manually.

## Troubleshooting

### Common Issues

1. **Docusaurus not starting**: Ensure Node.js 18+ is installed and run `npm install` again
2. **Missing ROS 2 dependencies**: Verify ROS 2 Humble is properly installed and sourced
3. **Python examples not working**: Ensure rclpy is available in your Python environment

### Getting Help

- Check the individual chapter pages for specific troubleshooting tips
- Refer to the official ROS 2 documentation for technical issues
- Review the Docusaurus documentation for site-specific issues
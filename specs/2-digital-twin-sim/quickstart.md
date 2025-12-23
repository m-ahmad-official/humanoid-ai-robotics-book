# Quickstart Guide: Digital Twin Simulation for Physical AI

## Prerequisites

- Node.js 18+ and npm/yarn
- Python 3.8+ with pip
- ROS 2 Humble Hawksbill installed
- Gazebo Garden or Fortress installed
- Unity 2022.3 LTS installed
- Basic ROS 2 knowledge

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
# Install required ROS 2 packages for simulation
# Install rclpy: sudo apt install python3-ros-rolling-rclpy
```

### 4. Install Gazebo (if not already part of ROS 2 installation)

```bash
# Follow Gazebo installation guide for your OS
# Verify installation: gazebo --version
```

### 5. Install Unity (for Unity simulation examples)

```bash
# Download and install Unity Hub
# Install Unity 2022.3 LTS through Unity Hub
# Install required Unity packages for robotics simulation
```

### 6. Start the Development Server

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### 7. Navigate the Educational Content

The educational content for Module 2 is organized as follows:

- **Introduction to Digital Twins**: Foundational concepts about digital twins and their role in Physical AI
- **Physics Simulation with Gazebo**: Understanding Gazebo physics, gravity, collisions, and simulating humanoid robots
- **High-Fidelity Interaction in Unity**: Learning about Unity rendering, human-robot interaction, and sensor simulation

## Running Simulation Examples

Each chapter includes practical simulation examples that demonstrate the concepts discussed:

1. Navigate to the specific chapter in the documentation
2. Find the simulation example section
3. The examples are designed to work with ROS 2, Gazebo, and Unity
4. Follow the instructions for each simulation environment

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
3. **Gazebo simulation not working**: Ensure Gazebo is properly installed and compatible with ROS 2
4. **Unity simulation examples**: Ensure Unity 2022.3 LTS is properly installed with robotics packages

### Getting Help

- Check the individual chapter pages for specific troubleshooting tips
- Refer to the official ROS 2 documentation for technical issues
- Review the Gazebo documentation for simulation-specific issues
- Check Unity documentation for rendering and interaction issues
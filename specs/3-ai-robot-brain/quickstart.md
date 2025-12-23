# Quickstart Guide: AI-Robot Brain with NVIDIA Isaac™

## Prerequisites

- Node.js 18+ and npm/yarn
- Python 3.8+ with pip
- ROS 2 Humble Hawksbill installed
- NVIDIA Isaac Sim installed (requires RTX GPU)
- Isaac ROS packages installed
- Nav2 navigation stack installed
- Robotics students familiar with ROS 2 and simulation concepts

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

### 3. Install Isaac Dependencies (for examples)

```bash
# Follow NVIDIA Isaac Sim installation guide for your OS and GPU
# Install Isaac ROS packages via ROS 2 package manager
# Install Nav2: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 4. Start the Development Server

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### 5. Navigate the Educational Content

The educational content for Module 3 is organized as follows:

- **Introduction to the AI-Robot Brain**: Foundational concepts about AI in humanoid robotics and Isaac's role in the ROS 2 ecosystem
- **Perception & Simulation with Isaac Sim**: Understanding Isaac Sim, photorealistic simulation, and synthetic data generation
- **Navigation & Intelligence**: Learning about Isaac ROS for perception/VSLAM and Nav2 integration for humanoid path planning

## Running Isaac Examples

Each chapter includes practical Isaac examples that demonstrate the concepts discussed:

1. Navigate to the specific chapter in the documentation
2. Find the Isaac example section
3. The examples are designed to work with Isaac Sim, Isaac ROS, and Nav2
4. Follow the instructions for each Isaac tool and integration

## Isaac System Requirements

### Hardware Requirements
- **GPU**: NVIDIA RTX series GPU (RTX 3070 or better recommended)
- **Memory**: 16GB RAM minimum (32GB recommended)
- **Storage**: 100GB free space for Isaac Sim and assets
- **Processor**: Multi-core processor (Intel i7 or AMD Ryzen 7+)

### Software Requirements
- **OS**: Ubuntu 22.04 LTS or Windows 10/11 (with WSL2)
- **CUDA**: CUDA 11.8+ compatible with your GPU
- **Isaac Sim**: 2023.1+ (download from NVIDIA Developer portal)
- **Omniverse**: Connected to Isaac Sim
- **ROS 2**: Humble Hawksbill

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
2. **Missing Isaac dependencies**: Verify Isaac Sim and ROS packages are properly installed
3. **GPU compatibility**: Ensure you have an NVIDIA RTX GPU with proper drivers
4. **Isaac Sim simulation not working**: Check Omniverse connection and GPU compatibility

### Isaac-Specific Issues

1. **Isaac Sim crashes**: Verify GPU driver and CUDA compatibility
2. **Synthetic data generation**: Ensure Isaac Sim is properly licensed and configured
3. **Isaac ROS integration**: Check ROS 2 workspace setup and package installations
4. **Nav2 humanoid navigation**: Verify humanoid robot model and controller configurations

### Getting Help

- Check the individual chapter pages for specific troubleshooting tips
- Refer to the official NVIDIA Isaac documentation for technical issues
- Review the ROS 2 and Nav2 documentation for navigation-specific issues
- Check Isaac Sim forums for simulation-related problems
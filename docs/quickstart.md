# Quickstart Guide: Physical AI & Humanoid Robotics Development Environment

## Prerequisites

### Hardware Requirements
- **Development Workstation**:
  - NVIDIA RTX GPU (RTX 4090, RTX A5000, or equivalent)
  - 32GB+ RAM
  - 1TB+ SSD storage
  - Ubuntu 22.04 LTS
- **Edge Computing**:
  - NVIDIA Jetson AGX Orin Developer Kit
  - 32GB RAM variant recommended
- **Robot Platform**:
  - Humanoid robot with ROS 2 support or proxy robot platform

### Software Requirements
- Ubuntu 22.04 LTS (both workstation and Jetson)
- ROS 2 Humble Hawksbill
- NVIDIA Container Toolkit
- Docker and Docker Compose
- Git and Git LFS

## Environment Setup

### 1. ROS 2 Humble Installation (Workstation)

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 in bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. NVIDIA Isaac Packages Installation

```bash
# Install NVIDIA drivers and CUDA
sudo apt install nvidia-driver-535
sudo apt install nvidia-cuda-toolkit

# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-* ros-humble-nitros-* ros-humble-isaac-ros-gxf

# Install Isaac Sim (requires NVIDIA Developer account)
# Download from https://developer.nvidia.com/isaac-sim
# Follow installation instructions for your platform
```

### 3. Gazebo Installation

```bash
# Install Gazebo Garden
sudo apt install ros-humble-gazebo-*

# Set Gazebo environment variables
echo "export GZ_SIM_RESOURCE_PATH=/usr/share/gazebo-11/worlds" >> ~/.bashrc
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" >> ~/.bashrc
source ~/.bashrc
```

### 4. Unity Robotics Setup

```bash
# Install Unity Hub and Unity 2022.3 LTS
# Download from https://unity.com/download

# Install Unity Robotics Package
# In Unity Hub, go to Packages > Add package from git URL
# Add: com.unity.robotics.ros-tcp-connector
# Add: com.unity.robotics.urdf-importer
```

### 5. Jetson Orin Setup

```bash
# Flash Jetson AGX Orin with JetPack 5.1.3 (includes ROS 2 Humble)
# Follow NVIDIA's Jetson documentation

# Install additional packages on Jetson
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-isaac-ros-* ros-humble-nitros-*

# Configure real-time capabilities
echo "ulimit -r 99" >> ~/.bashrc
echo "@realtime soft rtprio 99" >> /etc/security/limits.conf
echo "@realtime hard rtprio 99" >> /etc/security/limits.conf
```

## Basic ROS 2 Communication Test

### 1. Create a workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Test basic communication

```bash
# Terminal 1: Start a simple publisher
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Listen to messages
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

### 3. Check ROS 2 graph

```bash
# View active nodes
ros2 node list

# View active topics
ros2 topic list

# View topic types
ros2 topic list -t
```

## Simulation Environment Setup

### 1. Basic Gazebo Simulation

```bash
# Launch Gazebo with default world
source /usr/share/gazebo/setup.sh
gz sim

# Or launch with ROS 2 bridge
ros2 launch gazebo_ros empty_world.launch.py
```

### 2. Isaac Sim Basic Scene

```bash
# Launch Isaac Sim
isaac-sim.server.linux.sh

# Or through Docker (recommended)
docker run --gpus all -it --rm --network=host --shm-size=12gb --ulimit memlock=-1 --ulimit stack=67108864 -v {YOUR_ISAAC_PATH}:/isaac-sim workspace/isaac-sim:latest
```

## Basic Robot Control Example

### 1. Create a simple robot controller

```bash
cd ~/ros2_ws/src
ros2 pkg create --cpp-executable robot_controller my_robot_controller
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash
```

### 2. Run the controller

```bash
# In one terminal
ros2 run my_robot_controller robot_controller_exe

# Monitor with rqt
rqt
```

## Validation Steps

### 1. Verify ROS 2 Installation
```bash
# Check ROS 2 version
echo $ROS_DISTRO  # Should output 'humble'

# Check basic commands work
ros2 --help
```

### 2. Verify Simulation Environment
```bash
# Test Gazebo
gz --version

# Test Isaac Sim (if installed)
isaac-sim --version  # or check Docker container status
```

### 3. Verify AI Packages
```bash
# Check Isaac ROS packages
ros2 pkg list | grep isaac

# Check CUDA availability
nvidia-smi
```

## Troubleshooting

### Common Issues

1. **ROS 2 packages not found**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Gazebo fails to start**:
   - Check graphics drivers are properly installed
   - Ensure X11 forwarding if running remotely
   - Verify GPU compatibility

3. **Isaac Sim Docker fails**:
   - Verify NVIDIA Container Toolkit is installed
   - Check GPU access: `docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi`

4. **Real-time performance issues**:
   - Check CPU/GPU usage
   - Verify real-time kernel patches are applied
   - Adjust control loop frequencies as needed

### Performance Optimization

1. **For real-time control**:
   - Use real-time kernel patches
   - Configure CPU isolation
   - Set appropriate process priorities

2. **For AI inference**:
   - Use TensorRT optimization
   - Pre-load models in memory
   - Use appropriate batch sizes

3. **For simulation**:
   - Use dedicated GPU for rendering
   - Adjust physics update rates
   - Use simplified models for testing

## Next Steps

1. Follow Module 1: The Robotic Nervous System (ROS 2) to learn ROS 2 architecture for humanoid robotics
2. Explore Module 2: The Digital Twin (Gazebo & Unity) for simulation environments
3. Continue with Module 3: The AI-Robot Brain (NVIDIA Isaac) for perception and control
4. Complete Module 4: Vision-Language-Action (VLA) for multimodal AI integration
5. Build the capstone: Autonomous Humanoid system integrating all modules
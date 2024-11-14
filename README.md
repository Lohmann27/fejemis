# Cleaning Robot Workspace

This project contains the code for a cleaning robot developed with ROS2. The workspace includes several ROS2 packages, each responsible for different aspects of the robot's functionality, such as sensor management, control, simulation, and navigation.

## Table of Contents
- [Directory Structure](#directory-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Package Descriptions](#package-descriptions)
- [Contributing](#contributing)
- [License](#license)

## Directory Structure

Here’s an overview of the main directories in the `cleaning_robot_ws` workspace:

```
cleaning_robot_ws
│
├── build                      # ROS2 build directory
├── install                    # ROS2 install directory
├── log                        # ROS2 log directory
├── Pictures                   # Contains images related to the project
├── recordings                 # Stores recordings of simulation or real-world testing
├── src
│   ├── robot_control           # Responsible for robot control and movement
│   │   ├── config
│   │   ├── launch
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robot_description       # Contains the robot model (URDF/Xacro files)
│   │   ├── config
│   │   ├── launch
│   │   ├── xacro
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robot_in_practice       # Experimental or practical code for testing
│   │   ├── include
│   │   ├── src
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── robot_lidar             # Handles LiDAR data processing
│   │   ├── launch
│   │   ├── worlds
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── robot_simulation        # Provides simulation environments and configuration
│       ├── config
│       ├── launch
│       ├── maps
│       ├── scripts
│       ├── worlds
│       ├── CMakeLists.txt
│       └── package.xml
├── cleaning_robot_ws.code-workspace
├── How to start a launch.txt
├── Old Workspaces.code-workspace
└── Packages to install sudo.txt
```

## Installation

First, go to https://docs.ros.org/en/jazzy/ and install ROS2 Jazzy.

Make sure your system is up to date:

```sh
sudo apt update
sudo apt upgrade
```

Then install all the required packages:

### Install Core ROS2 Packages

```sh
sudo apt install -y \
  ros-jazzy-rclcpp \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-urdf \
  ros-jazzy-xacro \
  ros-jazzy-controller-manager \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-twist-mux
```

### Simulation Packages

```sh
sudo apt install -y ros-jazzy-ros-gz
```

### Navigation Packages

```sh
sudo apt install -y \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-recoveries \
  ros-jazzy-nav2-lifecycle-manager
```

### Additional Utilities

```sh
sudo apt install -y \
  ros-jazzy-rqt \
  ros-jazzy-rqt-robot-plugins \
  ros-jazzy-rqt-graph \
  ros-jazzy-rviz2
```

Clone this repository and initialize the ROS2 workspace:

```sh
git clone https://github.com/yourusername/cleaning_robot_ws.git
cd cleaning_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

To launch the entire system, use the provided launch files in each package. For example, to launch the simulation:

```sh
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_simulation main_simulation.launch.py
```

Replace `robot_simulation` with the specific package you want to use, and `main_simulation.launch.py` with other launch files as needed for different functionalities.

## Package Descriptions

### Controller Manager
The "Controller Manager" package manages multiple controllers on the robot. This package ensures that the robot's motors receive the correct commands. In this project, the Controller Manager is mainly used to control the robot in the simulated environment. It also allows the robot to be controlled by different controllers and switch seamlessly between different operational modes.

### Joy
The "Joy" package provides the necessary drivers and interfaces for integrating various joystick devices such as PS5, Xbox, or other controllers into the ROS2 system. This package is used as input for the "teleop_twist_joy" package.

### Teleop Twist Joy
The "Teleop Twist Joy" package enables manual control of the robot using a joystick. This package is used to translate joystick inputs into velocity commands, making it particularly useful during testing and debugging phases when manual control is required. Joystick control offers precise speed variation for the robot.

### Teleop Twist Keyboard
The "Teleop Twist Keyboard" package allows for controlling the robot via keyboard input. This is especially useful when a joystick is not available. Although keyboard control is not as precise as joystick control due to the lack of speed variation, it serves as a good fallback when no other input device is available.

### Twist Mux
"Twist Mux" is a package used to manage multiple velocity input sources. This package allows prioritizing different control inputs, such as joystick, keyboard, or autonomous navigation, so only one source is used at any given time, avoiding conflicts when multiple controllers are active.

### Gazebo Harmonic
The entire virtual environment for the robot is created using a simulation package called "Gazebo-Harmonic." This virtual world is used to test various aspects of the robot, such as sensor integration and driving, both manually and autonomously. These features of the robot are configured and tested in a safe environment before being implemented in the real world.

### SLAM
"SLAM" stands for Simultaneous Localization and Mapping, which allows the robot to create a map of its surroundings using a LiDAR scanner while determining its own position. SLAM is also required for autonomous navigation in an unknown environment.

### Nav2
"Nav2" or Navigation 2 is a navigation package that enables autonomous navigation within the map created by SLAM. Autonomous navigation includes path planning, obstacle avoidance, and goal execution in both known and unknown environments. Nav2 integrates with localization and control units, allowing the robot to move safely and autonomously towards a specific goal without colliding with obstacles such as walls or table legs.


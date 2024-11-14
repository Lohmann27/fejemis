# Cleaning Robot Workspace

This project contains the code for a cleaning robot developed with ROS2. The workspace includes several ROS2 packages, each responsible for different aspects of the robot's functionality, such as sensor management, control, simulation, and navigation.

## Table of Contents
- [Directory Structure](#directory-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Package Descriptions](#package-descriptions)


## Directory Structure

Here’s an overview of the main directories in the `cleaning_robot_ws` workspace:

```
cleaning_robot_ws
│
├── src
│   ├── robot_camera            # Manages camera data processing
│   ├── robot_control           # Responsible for robot control and movement
│   ├── robot_description       # Contains the robot model (URDF/Xacro files)
│   ├── robot_in_practice       # Experimental or practical code for testing
│   ├── robot_lidar             # Handles LiDAR data processing
│   └── robot_simulation        # Provides simulation environments and configuration
│
├── build                      # ROS2 build directory
├── install                    # ROS2 install directory
├── log                        # ROS2 log directory
├── Pictures                   # Contains images related to the project
└── recordings                 # Stores recordings of simulation or real-world testing
```

## Installation

First, go to https://docs.ros.org/en/foxy/ and install ROS2 Foxy.

Make sure your system is up to date:

```sh
sudo apt update
sudo apt upgrade
```

Then install all the required packages:

### Install Core ROS2 Packages

```sh
sudo apt install -y \
  ros-foxy-rclcpp \
  ros-foxy-std-msgs \
  ros-foxy-sensor-msgs \
  ros-foxy-geometry-msgs \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox \
  ros-foxy-tf2 \
  ros-foxy-tf2-ros \
  ros-foxy-robot-state-publisher \
  ros-foxy-joint-state-publisher \
  ros-foxy-urdf \
  ros-foxy-xacro \
  ros-foxy-controller-manager \
  ros-foxy-joy \
  ros-foxy-teleop-twist-joy \
  ros-foxy-teleop-twist-keyboard \
  ros-foxy-twist-mux
```

### Simulation Packages

```sh
sudo apt install -y ros-foxy-ros-gz
```

### Navigation Packages

```sh
sudo apt install -y \
  ros-foxy-nav2-map-server \
  ros-foxy-nav2-amcl \
  ros-foxy-nav2-controller \
  ros-foxy-nav2-planner \
  ros-foxy-nav2-recoveries \
  ros-foxy-nav2-lifecycle-manager
```

### Additional Utilities

```sh
sudo apt install -y \
  ros-foxy-rqt \
  ros-foxy-rqt-robot-plugins \
  ros-foxy-rqt-graph \
  ros-foxy-rviz2
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


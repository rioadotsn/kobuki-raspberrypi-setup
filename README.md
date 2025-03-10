# Kobuki Raspberry Pi Setup with ROS Noetic

This repository provides a step-by-step guide to configure a Raspberry Pi running Ubuntu 20.04 to connect and control a Kobuki robot using ROS Noetic.

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Installation Steps](#installation-steps)
4. [Testing and Verification](#testing-and-verification)
5. [Adding a Custom ROS Package](#adding-a-custom-ros-package)
6. [Kobuki on ROS Noetic](#Kobuki-on-ROS-Noetic)
---

## Introduction

The Kobuki robot base can be controlled via a Raspberry Pi running ROS Noetic. This guide covers:
- Installing ROS Noetic on Ubuntu 20.04.
- Setting up the necessary packages for Kobuki.
- Configuring the Raspberry Pi to communicate with the Kobuki robot base.

---

## Prerequisites

### Hardware
- Raspberry Pi 4b with Ubuntu 20.04 installed.
- Kobuki robot base.
- USB cable for Kobuki connection.
- Internet connection for package installation.

### Software
- Ubuntu 20.04 on Raspberry Pi.
- Git (for cloning repositories).
- Basic knowledge of Linux and ROS.

---

## Installation Steps

### Step 1: Install ROS Noetic
Follow the official instructions to install ROS Noetic:
1. Set up your sources list:
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

2. Add the ROS GPG key:
   ```bash
   sudo apt install curl # if you haven't already installed curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

3. Install ROS Noetic:
   ```bash
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   ```

4. Add ROS to your bash session:
   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
5. Install build dependencies:
   ```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```
   
6. Initialize rosdep:
   ```bash
   sudo rosdep init
   rosdep update
   ```

### Step 2: Set Up Catkin Workspace
Create and initialize a catkin workspace:
```bash
mkdir -p ~/your_ws/src
cd ~/your_ws/
catkin_make
cd src/
catkin_create_pkg my_robot_control rospy roscpp std_msgs
```

---

## Testing and Verification

### Step 1: Verify ROS Installation
Run the ROS master to ensure it's properly installed:
```bash
roscore
```

---

## Adding a Custom ROS Package

1. **Write Your C++ Node**
   Create a `src/hello_world.cpp` file inside the `my_robot_control` package:
   ```cpp
   #include <ros/ros.h>

   int main(int argc, char** argv) {
       ros::init(argc, argv, "hello_world_node");
       ros::NodeHandle nh;

       ROS_INFO("Hello, ROS World!");

       ros::spin();
       return 0;
   }
   ```

2. **Update CMakeLists.txt**
   Edit the `CMakeLists.txt` file in the `my_robot_control` package to include your node:
   ```cmake
   add_executable(hello_world_node src/hello_world.cpp)
   target_link_libraries(hello_world_node ${catkin_LIBRARIES})
   ```

3. **Build the Package**
   Go back to your workspace root and build:
   ```bash
   cd ~/your_ws
   catkin_make
   ```

4. **Run Your Node**
   Source your workspace and launch the node:
   ```bash
   source devel/setup.bash
   rosrun my_robot_control hello_world_node
   ```
5. **Write Your Python Node**
   Create a `scripts/hello_world.py` file inside the `my_robot_control` package:
   ```python
   #!/usr/bin/env python3
   import rospy

   if __name__ == "__main__":
       rospy.init_node("hello_world_node")
       rospy.loginfo("Hello, ROS World from Python!")
       rospy.spin()
   ```

6. **Update CMakeLists.txt**
   Modify the `CMakeLists.txt` file to install the Python script:
   ```cmake
   catkin_install_python(PROGRAMS
     scripts/hello_world.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

7. **Make the Script Executable**
   Set the appropriate permissions for the Python script:
   ```bash
   chmod +x scripts/hello_world.py
   ```

8. **Run Your Python Node**
   Source your workspace and launch the node:
   ```bash
   source devel/setup.bash
   rosrun my_robot_control hello_world.py
   ```
## Kobuki on ROS Noetic
   https://github.com/rioadotsn/kobuki_custom.git

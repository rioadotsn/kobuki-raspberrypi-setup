# kobuki-raspberrypi-setup
A guide to set up Raspberry Pi with Kobuki robot
# Kobuki Raspberry Pi Setup with ROS Noetic

This repository provides a step-by-step guide to configure a Raspberry Pi running Ubuntu 20.04 to connect and control a Kobuki robot using ROS Noetic.

## Introduction

The Kobuki robot base can be controlled via a Raspberry Pi running ROS Noetic. This guide covers:
- Installing ROS Noetic on Ubuntu 20.04.
- Setting up the necessary packages for Kobuki.
- Configuring the Raspberry Pi to communicate with the Kobuki robot base.

---

## Prerequisites

### Hardware
- Raspberry Pi 4 with Ubuntu 20.04 installed.
- Kobuki robot base.
- USB cable for Kobuki connection.
- Internet connection for package installation.

### Software
- Ubuntu 20.04 on Raspberry Pi.
- Git (for cloning repositories).
- Basic knowledge of Linux and ROS.

---

## Installation Steps

### Step 1: Update System

  sudo apt update

### Step 2: Install ROS Noetic

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  sudo apt install curl # if you haven't already installed curl

  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  sudo apt update

  sudo apt install ros-noetic-desktop-full

  source /opt/ros/noetic/setup.bash

  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  
  source ~/.bashrc

  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

  sudo rosdep init
  
  rosdep update

### Step 3: Set Up Workspace

  mkdir -p ~/your_ws/src
  
  cd ~/your_ws/
  
  catkin_make

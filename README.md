#PX4 ROS 2 Gazebo QGroundControl Teleoperation and Obstacle Avoidance Simulation
Overview

This repository provides a complete ROS 2–based teleoperation and autonomous control framework for a PX4 SITL (Software-In-The-Loop) drone simulation.
The system integrates PX4 Autopilot, Gazebo Sim, ROS 2, and QGroundControl to enable:

Realistic UAV simulation in Gazebo

Telemetry monitoring and manual control via QGroundControl

Keyboard and joystick-based teleoperation using ROS 2

LiDAR-based obstacle detection and avoidance

PX4–ROS 2 communication using Micro XRCE-DDS

The setup closely mirrors real PX4 hardware workflows and is intended for research, learning, and pre-flight algorithm validation.

Objective

The primary objective of this project is to:

Create a ROS 2–based teleoperation system for a PX4 SITL drone

Run UAV simulation in Gazebo Sim

Monitor flight state and telemetry using QGroundControl

Control the drone using keyboard or joystick via ROS 2 topics

Implement LiDAR-based obstacle avoidance within the simulation environment

System Requirements
Operating System

Ubuntu 22.04 LTS

Software

ROS 2 Humble Hawksbill

PX4 Autopilot (SITL)

Gazebo Sim

QGroundControl

Micro XRCE-DDS Agent

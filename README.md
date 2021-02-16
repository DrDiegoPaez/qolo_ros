## Ros Interface for controlling the robot Qolo @LASA
This repository is used for integrating the main controller on the robot Qolo [1], all sensors, obstacle avoidance, and high-level controllers with the low-level motor control of the robot.
Package contaiing Qolo Controllers:

User Iterface:
  * Embodided control with Hands-free Navigation [2] 
  * Joystick remote control with html interface
  
Controllers:
  * Obstacle Avoidance with RDS [4]
  * Compliant control [REF]
  * Modulated Dynamical Systems based control [REF]
  
Communication: 
  * Controllers for real-robot Qolo.
  * Controllers version for the CrwodBot simulator version of Qolo. [REF]
  
![Alt text](/visualisation/qolo_images/qolo_sim.png?raw=true "Qolo version @LASA-EPFL and CrowdBot Simulator version.")

Mantainer: Dr. Diego Paez G.
Start Date: 2019-01-01

Last Update: 2021-02-15

## Prerequisites

Ubnutu 16.04
ROS kinetic
python 2.7
python-pip

## Using on the real Qolo:
Install Ubuntu Kernel for Upboard: https://wiki.up-community.org/Ubuntu
enablel spi port on UP board
	https://wiki.up-community.org/Pinout_UP2#SPI_Ports

Enable the HAT functionality from userspace
	https://wiki.up-community.org/Ubuntu

Install MRAA library: https://github.com/intel-iot-devkit/mraa
PYYAML: For FT model

Follow the installation guide: 
https://github.com/DrDiegoPaez/qolo_ros/blob/master/install_scripts/Install_instructions.txt

## Related packages:

Obstacle avoidance for tight shape and non-holonomic constraints (used in shared control):
https://github.com/epfl-lasa/rds

Obstacle avoidance based on dynamical systems:
https://github.com/epfl-lasa/qolo_modulation
https://github.com/epfl-lasa/dynamic_obstacle_avoidance/

Pybullet simulation for collisions and compliant control with Qolo and other robots:
https://github.com/epfl-lasa/human-robot-collider

## Usage Guides:

Launch sequence guide for real Qolo:
https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/Guide-CrowdBotUnity-ROS

General Usage guide for CrowdBot Simulator:
https://github.com/DrDiegoPaez/qolo_ros/blob/master/qolo_package/launch/launch_sequences.txt

## References for citations:
Qolo Design:
[1] Paez-Granados, D. F., Kadone, H., & Suzuki, K. (2018). Unpowered Lower-Body Exoskeleton with Torso Lifting Mechanism for Supporting Sit-to-Stand Transitions. IEEE International Conference on Intelligent Robots and Systems, 2755–2761. https://doi.org/10.1109/IROS.2018.8594199

Qolo Hands-free control:
[2] Chen, Y., Paez-Granados, D., Kadone, H., & Suzuki, K. (2020). Control Interface for Hands-free Navigation of Standing Mobility Vehicles based on Upper-Body Natural Movements. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS-2020).

Qolo shared control:
[3] Paez-Granados, D., Billard, A., & Suzuki, K. (2020). Materializing Personal Standing Mobility from Design to Shared Control. In ETH Zurich Rehabilitation Engineering Lab (Ed.), CYBATHLON Symposium. Cybathlon. https://cybathlon-symposium.ethz.ch

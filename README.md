## Ros Interface for controlling the robot Qolo @LASA
This repository is used for integrating the main controller on the robot Qolo [1], all sensors, obstacle avoidance, and high-level controllers with the low-level motor control of the robot [3].
Package containing the following:

User Iterface:
  * Embodided control with Hands-free Navigation [2] 
  * Joystick remote control with html interface
  
Controllers:
  * Redirecting Driver Support (RDS): Obstacle Avoidance for non-holonomic robots [P1]
  * Modulated Dynamical Systems based obstacle avoidance [P2]
  * Compliant control [P3]
  
Communication: 
  * Controllers for real-robot Qolo.
  * Controllers version for the CrowdBot simulator version of Qolo. [REF]
  
![Alt text](/visualisation/qolo_images/qolo_sim.png?raw=true "Qolo version @LASA-EPFL and CrowdBot Simulator version.")

Mantainer: Dr. Diego Paez G.
Start Date: 2019-01-01

Last Update: 2021-02-15

## Prerequisites

Ubnutu 16.04 or 18.04
ROS kinetic / melodic
python 2.7
python-pip
rds_network_ros # For shared control --> See related package below

# For Visualization in RVIZ:
jsk_rviz_plugin
	* sudo apt-get install -y ros-VERSION-jsk-visualization
rwth_messages (for people tracker visualization -- See below in Visualization section)


## Installation on the real Qolo:
Install Ubuntu Kernel for Upboard: https://wiki.up-community.org/Ubuntu
enablel spi port on UP board
	https://wiki.up-community.org/Pinout_UP2#SPI_Ports

Enable the HAT functionality from userspace
	https://wiki.up-community.org/Ubuntu

Install MRAA library: https://github.com/intel-iot-devkit/mraa
PYYAML: For FT model

Follow the installation guide: 
https://github.com/DrDiegoPaez/qolo_ros/blob/master/install_scripts/Install_instructions.txt

## Usage Guides:

Qolo overall system guide:

https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/Qolo_LASA_Specs.pdf

Launch sequence guide for real Qolo:

https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/launch_sequences.txt

Remote Joystick setup and usage:

https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/ROS-joystick-guide.txt

General Usage guide for CrowdBot Simulator:

https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/launch_sequence-CrowdBotUnity-ROS.txt

## Visualization Guide:

Install the packages: {also found online}
> visualisation/messages.zip
> visualisation/spencer_tracking_rviz_plugin.zip
* Only compatible with ROS-kinetic or ROS-melodic (Ubuntu 16 or 18)

catkin_make of the ros package on your wrokspace
source your workspace
roslaunch qolo_ros rviz.launch

You can select among different presets for rviz depending on the available data (LIDARs / sensory board / force sensors / people tracker)

## Related packages:

[P1] Obstacle avoidance for tight shape and non-holonomic constraints (used in shared control): [3] 

https://github.com/epfl-lasa/rds

[P2] Obstacle avoidance based on dynamical systems:

https://github.com/epfl-lasa/qolo_modulation
https://github.com/epfl-lasa/dynamic_obstacle_avoidance/

[P3] Pybullet simulation for collisions and compliant control with Qolo and other robots [4]:

https://github.com/epfl-lasa/human-robot-collider

[P4] CrowdBot Simulator Package:

https://gitlab.inria.fr/CrowdBot/CrowdBotUnity

[P5] Qolo 3D model for simulation files:

https://github.com/FabienGrzeskowiakInria/CrowdBot_robots


## References:
Qolo Design:

> [1] Paez-Granados, D. F., Kadone, H., & Suzuki, K. (2018). Unpowered Lower-Body Exoskeleton with Torso Lifting Mechanism for Supporting Sit-to-Stand Transitions. IEEE International Conference on Intelligent Robots and Systems, 2755–2761. https://doi.org/10.1109/IROS.2018.8594199

Qolo Hands-free control:

> [2] Chen, Y., Paez-Granados, D., Kadone, H., & Suzuki, K. (2020). Control Interface for Hands-free Navigation of Standing Mobility Vehicles based on Upper-Body Natural Movements. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS-2020). https://doi.org/10.1109/IROS45743.2020.9340875

Qolo shared control:

> [3] Gonon, D. J., Paez-Granados, D., & Billard, A. (2021). Reactive Navigation in Crowds for Non-holonomic Robots with Convex Bounding Shape. IEEE Robotics and Automation Letters, 6(3), 4728–4735. https://doi.org/10.1109/LRA.2021.3068660

> [4] Paez-Granados, D., Billard, A., & Suzuki, K. (2020). Materializing Personal Standing Mobility from Design to Shared Control. In ETH Zurich Rehabilitation Engineering Lab (Ed.), CYBATHLON Symposium. Cybathlon. https://cybathlon-symposium.ethz.ch

Crowd Navigation & safety assessment:

> [5] Paez-granados, D., Gonon, D., Salvini, P., & Billard, A. (2020). Physical Safety in Collisions Between Robots and Pedestrians. IEEEE International Conference on Robot and Human Interaactive Communication (ROMAN-2020) Workshop on Robots from Pathways to Crowds, 1–2. https://doi.org/10.13140/RG.2.2.28087.55209

> [6] Grzeskowiak, F., Gonon, D., Dugas, D., Paez-granados, D., Chung, J. J., Nieto, J., Siegwart, R., Billard, A., Babel, M., & Pettr, J. (2021). Crowd against the machine : A simulation-based benchmark tool to evaluate and compare robot capabilities to navigate a human crowd. IEEE International Conference on Robotics and Automation (ICRA-2021).

Compliant Control:

> [7] Paez-Granados, D., Gupta, V., & Billard, A. (2021). Unfreezing Social Navigation : Dynamical Systems based Compliance for Contact Control in Robot Navigation. Robotics Science and Systems (RSS) - Workshop on Social Robot Navigation, 1(1), 1–4.http://infoscience.epfl.ch/record/287442?&ln=en.  https://youtu.be/y7D-YeJ0mmg

**Contact**: 
[Dr. Diego Paez]
https://diegofpaez.wordpress.com/

**Acknowledgments**
This project was partially founded by:
> The EU Horizon 2020 Project CROWDBOT (Grant No. 779942): http://crowdbot.eu

> The Toyota Mobility Foundation (TMF) through the Grant: Mobility Unlimited Challenge 2019: https://mobilityunlimited.org

> Grant-in-Aid for Scientific Research from the Ministry of Education, Culture, Sports, Science and Technology of (MEXT) Japan: http://www.ai.iit.tsukuba.ac.jp/research/046.html

# Launch Sequence for experiments in Qolo robot #

ssh -X qolo@192.168.13.110
main UpBoard 110
Frontal Lidar 120
Hasler 130
Nvidia board 200
	pass: ....

# == Experiments for OA recordings - Including Collisions == #

#### Establish 4 connetcions to main UpBoard 110
#### Establish 1 connetcion to Frontal Upboard 120
#### Establish 2 connetcion to Frontal Upboard 200

### Check that all dates and times are the same on all PCs ###
	date
### If a PC is out of sync it might be that the internal NTP server went down ###
	Manual setup:
	sudo /etc/init.d/ntp restart
	sudo ntpd -q 192.168.13.110

**1. 110 terminal:**
	rosclean purge -y
	roscore

**2. 110 terminal:rear LIDAR**
	cd ~/catkin_ws/
	. devel/setup.bash
	rosrun qolo rear_lidar2lrf.sh

	**Alternatively:**
	roslaunch qolo rear_lidar-cloud.launch

**3. 120 terminal: Launching Front Lidar and Low-level avoidance (RDS)**
	cd ~/autonomy_ws/
	. devel/setup.bash
	rosrun rds_ros rds_lidar2lrf.sh

	# Optional without laser_scan_all
		roslaunch rds_ros rds_front_lrf.launch det:="true"

**4. 200: Localization from T265 camera**
	rosclean purge -y
	cd /ssd_nvidia/autonomy_ws
	. devel/setup.bash
	roslaunch realsense2_camera qolo_localization_t265.launch

%% --> WAIT 15seconds  BEFORE THE NEXT CAMERA LAUNCH


**5. Nvidia-200 terminal:  Start front-left camera**
	cd /ssd_nvidia/autonomy_ws
	. devel/setup.bash
	roslaunch realsense2_camera qolo_left_camera.launch

**6. 110 terminal: Starting main Qolo Control Node**
	sudo -s
	cd ~/catkin_ws/
	. devel/setup.bash
	rosrun qolo compliant_remote_shared_qolo.sh

**7. Nvidia-200 terminal:  Start People tracker**
	cd ~/tracker_ws
	. /ssd_nvidia/venv_sensing/bin/activate
	. devel/setup.bash
	roslaunch rwth_crowdbot_launch qolo_onboard.launch trt:=true

**7. 200: Rosbag Recording**
	cd /ssd_nvidia/data/irl_obstacles/
	rosbag record --duration=30s /tf /tf_static /diagnostics /front_lidar/scan /front_lidar/scan_all /front_lidar/velodyne_points /rear_lidar/velodyne_points /rear_lidar/scan /rear_lidar/scan_all /joint_states /qolo/compliance/svr /qolo/emergency /qolo/odom /qolo/pose2D /qolo/remote_commands /qolo/twist /rds_to_gui /rokubi_node_front/ft_sensor_measurements /rosout /rosout_agg /t265/accel/imu_info /t265/accel/sample /t265/gyro/imu_info /t265/gyro/sample /t265/odom/sample

	/t265/fisheye1/camera_info /t265/fisheye1/image_raw /t265/fisheye2/camera_info /t265/fisheye2/image_raw

**8. 110 terminal: DS trajectory (20 m ahead)**
	####### For RDS trajectory #######
	Main-pc-110 terminal:
	ssh qolo@192.168.13.110
	cd ~/catkin_ws/
	. devel/setup.bash
	rosrun qolo ds_trajectory.py

	####### For MDS trajectory #######
	ssh qolo@192.168.13.120
	cd ~/autonomy_ws/
	. devel/setup.bash
	rosrun qolo_modulation qolo_modulation_ros_controller.py

**9. 200 terminal: Nvidia board DATA recording**
    Choose a recording at the bottom of the document that suits the test (the following is the simplest LRF + tracker + qolo)

	cd /ssd_nvidia/data/crowdbot_2021/29_11_2021/shared_control
	rosbag record --duration=3m /chatter /tf /tf_static qolo/odom /qolo/pose2D /qolo/twist /qolo/remote_commands /qolo/user_commands qolo/compliance/svr /qolo/emergency /front_lidar/velodyne_points /front_lidar/scan /front_lidar/scan_all /rear_lidar/velodyne_points /rear_lidar/scan /rear_lidar/scan_all /camera_left/color/image_raw /camera_left/color/camera_info /camera_left/depth/camera_info /camera_left/depth/image_rect_raw /camera_left/depth/color/points /camera_left/aligned_depth_to_color/camera_info /camera_left/aligned_depth_to_color/image_raw /ground_plane /ground_plane_visual_marker /image_with_bounding_boxes /darknet_ros/bounding_boxes /darknet_ros/detection_image /detected_persons/yolo /detected_persons_synchronized /diagnostics /drow_detected_persons_front /drow_detected_persons_rear /map /map_metadata /move_base_simple/goal /poseupdate /rwth_tracker/pedestrian_array /rwth_tracker/tracked_persons /rds_to_gui /t265/odom/sample /rokubi_node_front/ft_sensor_measurements /t265/accel/imu_info /t265/accel/sample /t265/gyro/imu_info /t265/gyro/sample

## Local (machine) Setup for visualization ##

Configure remote master:
	export ROS_MASTER_URI=http://192.168.13.110:11311

Configure Local ROS IP:
	export ROS_IP=192.168.13.XX --> (XX check local IP given by QoloNet)

**10. Visualization:**
	cd ~/qolo_ws/
	. devel/setup.bash
	roslaunch qolo rviz.launch

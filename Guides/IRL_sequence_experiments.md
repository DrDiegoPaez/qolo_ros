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

1. 110 terminal:
	rosclean purge -y
	roscore

**2. 110 terminal:rear LIDAR**
	cd ~/catkin_ws/
	. devel/setup.bash
	rosrun qolo rear_lidar2lrf.sh
	**Alternatevely:**
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

**5. 110 terminal: Starting main Qolo Control Node**
	sudo -s
	cd ~/catkin_ws/
	. devel/setup.bash
	rosrun qolo compliant_remote_shared_qolo.sh


**6. 200: Rosbag Recording**
	cd /ssd_nvidia/data/irl_obstacles/
	rosbag record --duration=30s /tf /tf_static /diagnostics /front_lidar/scan /front_lidar/scan_all /front_lidar/velodyne_points /rear_lidar/velodyne_points /rear_lidar/scan /rear_lidar/scan_all /joint_states /qolo/compliance/svr /qolo/emergency /qolo/odom /qolo/pose2D /qolo/remote_commands /qolo/twist /rds_to_gui /rokubi_node_front/ft_sensor_measurements /rosout /rosout_agg /t265/accel/imu_info /t265/accel/sample /t265/gyro/imu_info /t265/gyro/sample /t265/odom/sample

	/t265/fisheye1/camera_info /t265/fisheye1/image_raw /t265/fisheye2/camera_info /t265/fisheye2/image_raw

**7. 110 terminal: DS trajectory**
	cd ~/catkin_ws/
	. devel/setup.bash
	rosrun qolo ds_trajectory.py
	
**8. Visualization:**
	cd ~/qolo_ws/
	. devel/setup.bash
	roslaunch qolo rviz.launch

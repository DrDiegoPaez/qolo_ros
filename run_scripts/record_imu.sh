#!/bin/bash
​
#----- Ctrl-C stop -----
trap "exit" INT TERM ERR
trap "kill -INT 0" EXIT
​
#----- Get Test Number -----
TEST_NO=0
#while [ -d "csv_logs/test${TEST_NO}" ]; do
#    TEST_NO=$(( $TEST_NO + 1 ))
#done
LOG_FOLDER="$(pwd)/csv_logs/test${TEST_NO}"
eval "mkdir -p ${LOG_FOLDER}"
echo "Current Test Number : ${TEST_NO}"
eval ". devel/setup.bash" 
​
#----- Launch and record force sensors -----
#eval "./src/rokubimini_interface/run_rokubimini_ros.sh -f ${LOG_FOLDER}&"
#PID_LIST+="$! "
​
#sleep 5
​
#----- Launch and record realsense camera -----
echo "Launching RealSense Camera..."
eval "roslaunch realsense2_camera rs_qolo_front.launch \
    &> /dev/null &"
PID_LIST+="$! "
​
#eval "rosbag record -q \
#    -O ${LOG_FOLDER}/camera \
#    -e '/camera_rear/(.*)' \
#    &> /dev/null &"
# PID_LIST+="$! "

eval "rostopic echo -p /camera/accel/sample \
    &> ${LOG_FOLDER}/imu/accel.csv &"
PID_LIST+="$! "
eval "rostopic echo -p /camera/gyro/sample \
    &> ${LOG_FOLDER}/imu/gyro.csv &"
PID_LIST+="$! "
​
sleep 5
​
# #----- Launch Rear Lidar  -----
# echo "Launching Velodyne Rear LIDAR..."
# eval "roslaunch rds_ros rds_rear_lrf.launch"
# PID_LIST+="$! "
# ​
# sleep 5
​
# Wait till all pids to be finished or killed
echo "All PIDs : ${PID_LIST}"
for PID in ${PID_LIST[@]};do
    wait $PID

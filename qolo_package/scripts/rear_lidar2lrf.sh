#!/bin/bash

#----- Colored Terminal -----
NORMAL="\e[0m"
IMP_INFO="\e[34;1m"
IMP_RED="\e[31;1m"
IMP_GREEN="\e[32;1m"

#----- Ctrl-C stop -----
_kill() {
    echo -e "${IMP_RED}Killing all subprocesses${NORMAL}"
    for PID in ${PID_LIST[@]};do
        kill -INT $PID
    done
}
trap "exit" INT TERM ERR
trap _kill EXIT

cd ~/catkin_ws/
    . devel/setup.bash


# #----- Launch qolo's odometry -----
# echo -e "${IMP_INFO}Launching QOLO Odometry Node...${NORMAL}"
# # eval "roslaunch qolo compliance_qolo.launch log_folder:=${LOG_FOLDER} &"
# # eval "rosrun qolo t265_pose_qolo.py "
# eval ". devel/setup.bash"
# eval "roslaunch qolo odometry_t265.launch"
# PID_LIST+="$! "
# sleep 5
    
#----- Launch Rear Lidar  -----
echo -e "${IMP_INFO}Launching REAR LIDAR...${NORMAL}"
eval ". devel/setup.bash"
eval "roslaunch qolo rear_lidar-cloud.launch &"
PID_LIST+="$! "

sleep 5

# #----- Launch LIDAR-2-LRF Node -----
echo -e "${IMP_INFO}Launching LIDAR-2-LRF Node...${NORMAL}"
eval "roslaunch pointcloud_to_laserscan lidar2lrf_rear.launch"
PID_LIST+="$! "
sleep 3


# Wait till all pids to be finished or killed
echo -e "${IMP_GREEN}All PIDs : ${PID_LIST}${NORMAL}"
for PID in ${PID_LIST[@]};do
    wait $PID
done

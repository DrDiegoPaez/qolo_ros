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

#----- Get Test Number -----
TEST_NO=0
while [ -d "csv_logs/bug/remote${TEST_NO}" ]; do
   TEST_NO=$(( $TEST_NO + 1 ))
done
LOG_FOLDER="$(pwd)/csv_logs/bug/remote${TEST_NO}"
eval "mkdir -p ${LOG_FOLDER}/compliance"
echo -e "${IMP_INFO}Current Test Number : ${TEST_NO}${NORMAL}"

#----- Launch and record force sensors -----
# echo -e "${IMP_INFO}Launching FT Sensors...${NORMAL}"
# eval "source /home/qolo/collision_ws/devel/setup.bash"
# eval ". /home/qolo/collision_ws/src/rokubimini_interface/run_rokubimini_ros.sh -f ${LOG_FOLDER} &"
# PID_LIST+="$! "
# sleep 5

#----- Launch Rear Lidar  -----
echo -e "${IMP_INFO}Launching REAR LIDAR...${NORMAL}"
eval ". devel/setup.bash"
eval "roslaunch qolo rear_lidar-cloud.launch &"
PID_LIST+="$! "

sleep 5

#----- Launch qolo control -----
echo -e "${IMP_INFO}Launching QOLO Control Node...${NORMAL}"
eval ". devel/setup.bash"
eval "roslaunch qolo bug_remote_compliant_qolo.launch log_folder:=${LOG_FOLDER} &"
PID_LIST+="$! "
sleep 15

# #----- Launch qolo's odometry -----
echo -e "${IMP_INFO}Launching QOLO Odometry Node...${NORMAL}"
eval "roslaunch qolo odometry_t265.launch"
PID_LIST+="$! "
sleep 3

# Wait till all pids to be finished or killed
echo -e "${IMP_GREEN}All PIDs : ${PID_LIST}${NORMAL}"
for PID in ${PID_LIST[@]};do
    wait $PID
done

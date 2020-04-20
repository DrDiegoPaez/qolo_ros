#!/bin/bash

#----- Ctrl-C stop -----
trap "exit" INT TERM ERR
trap "kill -INT 0" EXIT

#----- Get Test Number -----
TEST_NO=0
while [ -d "csv_logs/test${TEST_NO}" ]; do
   TEST_NO=$(( $TEST_NO + 1 ))
done
LOG_FOLDER="$(pwd)/csv_logs/test${TEST_NO}"
eval "mkdir -p ${LOG_FOLDER}"
echo "Current Test Number : ${TEST_NO}"
eval ". devel/setup.bash" 


# #----- Launch and record realsense camera -----
# echo "Launching RealSense Camera..."
# eval "roslaunch realsense2_camera rs_qolo_rear.launch \
#     &> /dev/null &"
# PID_LIST+="$! "
# sleep 1

#----- Launch and record force sensors -----
eval "/home/qolo/collision_test/src/rokubimini_interface/run_rokubimini_ros.sh -f ${LOG_FOLDER}&"
sleep 5
PID_LIST+="$! "


# eval "rosbag record -a \
    # -O ${LOG_FOLDER}/compliant_test \
    # &> /dev/null &"
 # PID_LIST+="$! "

# Wait till all pids to be finished or killed
echo "All PIDs : ${PID_LIST}"
for PID in ${PID_LIST[@]};do
    wait $PID
done

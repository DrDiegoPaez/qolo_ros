#!/bin/bash
#----- Ctrl-C stop -----
_kill() {
    echo "Killing all subprocesses"
    for PID in ${PID_LIST[@]};do
        kill -INT $PID
    done
}
trap "exit" INT TERM ERR
trap _kill EXIT

#----- Get Test Number -----
TEST_NO=0
while [ -d "csv_logs/test${TEST_NO}" ]; do
   TEST_NO=$(( $TEST_NO + 1 ))
done
LOG_FOLDER="$(pwd)/csv_logs/test${TEST_NO}"
eval "mkdir -p ${LOG_FOLDER}/compliance"
echo "Current Test Number : ${TEST_NO}"

#----- Launch and record force sensors -----
echo "Launching FT Sensors..."
eval "source /home/qolo/collision_ws/devel/setup.bash"
eval ". /home/qolo/collision_ws/src/rokubimini_interface/run_rokubimini_ros.sh -f ${LOG_FOLDER} &"
PID_LIST+="$! "
sleep 5

#----- Launch qolo control -----
eval ". devel/setup.bash"
eval "roslaunch qolo compliance_qolo.launch &"
PID_LIST+="$! "
sleep 15

echo ":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
echo "Starting recording compliance test..."
echo ":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"

eval "rostopic echo -p /qolo/compliance/raw \
    &> ${LOG_FOLDER}/compliance/raw.csv &"
PID_LIST+="$! "
eval "rostopic echo -p /qolo/compliance/svr \
    &> ${LOG_FOLDER}/compliance/svr.csv &"
PID_LIST+="$! "
eval "rostopic echo -p /qolo/compliance/bumper_loc \
    &> ${LOG_FOLDER}/compliance/bumper_loc.csv &"
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

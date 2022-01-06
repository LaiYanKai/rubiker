
#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

#sudo ip ad add 10.0.0.1/24 dev eth0
export ROS_HOSTNAME=192.168.105.117
export ROS_MASTER_URI=http://192.168.105.117:11311

# MTR PIN_K(BCM) PIN_W(BCM) PIN_Y(WPI/Name) PIN_B(WPI/Name) KP KI KD
export MotorLW="LW 15 14 4 1 2.2 0.1 0.8"
export MotorRH="RH 25 24 27 26 1.0 0.1 0.8"
export MotorLH="LH 7 8 29 28 1.0 0.1 0.8"
export MotorRW="RW 6 5 25 24 2.0 0.01 1.0"
export instructions="L"
#BfTlRF -- 13 sec
#tBDflRbrd -- 30 sec
#lRTbdLFL -- 30 sec
#tfTrfbDl -- 36 sec

roslaunch rubiker rubiker.launch

#echo "================= RESETING ===================="
#source reset.sh

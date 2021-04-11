#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

sudo ip ad add 10.0.0.1/24 dev eth0
# MTR PIN_K(BCM) PIN_W(BCM) PIN_Y(WPI/Name) PIN_B(WPI/Name) KP KI KD
export MotorLW="LW 14 15 1 4 2.0 0.01 1.0"
export MotorRH="RH 8 7 5 6 1.0 0.1 0.8"
export MotorLH="LH 1 12 27 28 1.0 0.1 0.8"
export MotorRW="RW 27 22 12 13 1.8 0.1 0.8"
export instructions="BfrrTlF"
#BfTlRF -- 13 sec
#tBDflRbrd -- 30 sec
#lRTbdLFL -- 30 sec
#tfTrfbDl -- 36 sec

roslaunch rubiker rubiker.launch

#echo "================= RESETING ===================="
#source reset.sh

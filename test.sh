
#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

#sudo ip ad add 10.0.0.1/24 dev eth0

# MTR PIN_K(BCM) PIN_W(BCM) PIN_Y(WPI/Name) PIN_B(WPI/Name) KP KI KD
#export Motor="LW 14 15 4 1 1 0.1 0.8"
export Motor="RH 25 24 26 27 1.0 0.1 0.8"
#export Motor="LH 7 8 28 29 1.0 0.1 0.8"
#export Motor="RW 5 6 25 24 1.0 0.01 1.0"
#export instructions="L"
#BfTlRF -- 13 sec
#tBDflRbrd -- 30 sec
#lRTbdLFL -- 30 sec
#tfTrfbDl -- 36 sec

roslaunch rubiker test.launch

#echo "================= RESETING ===================="
#source reset.sh

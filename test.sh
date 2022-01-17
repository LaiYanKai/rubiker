
#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

#sudo ip ad add 10.0.0.1/24 dev eth0

# MTR PIN_K(BCM) PIN_W(BCM) PIN_Y(BCM) PIN_B(BCM) KP KI KD AcceptableError Verbose
#export Motor="LW 14 15 23 18 2.0 0.001 3.0 5 1"
#export Motor="RH 25 24 12 16 1.0 0.01  2.0 5 1"
export Motor="LH 17  27  20 21 1.0 0.01  2.0 5 1"
#export Motor="RW 6  5  19 26 2.0 0.001 3.0 5 1"

roslaunch rubiker test.launch

#echo "================= RESETING ===================="
#source reset.sh

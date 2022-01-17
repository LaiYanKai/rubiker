
#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

sudo ip ad add 10.0.0.1/24 dev eth0
sudo ip link set dev eth0 up
export ROS_HOSTNAME=10.0.0.1
export ROS_MASTER_URI=http://10.0.0.1:11311

# 		MTR 	K(BCM)	W(BCM)	Y(BCM) B(BCM) 	KP 	KI 	KD 	ErrThrs	Verbose
export MotorLW="LW	15	14	18	23	2.0	0.001	3	5	0"
export MotorRH="RH	25	24	12	16	2.0	0.001	2	5	0"
export MotorLH="LH	17	27	20	21	2.5	0.005	3	5	0"
export MotorRW="RW	6	5	19	26	2.0	0.001	3	5	0"
# 		Instructions	WRISTAW H{OC} 	TIMEOUT OFFW{L,R,F,B}	P{U,M,D}
export master="	pCP		540	0 550	1.00	90 90 90 90	0 -100 -480"
#bdLFt
#BfTlRF -- 13 sec
#tBDflRbrd -- 30 sec
#lRTbdLFL -- 30 sec
#tfTrfbDl -- 36 sec

roslaunch rubiker rubiker.launch

#echo "================= RESETING ===================="
#source reset.sh

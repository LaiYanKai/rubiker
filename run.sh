#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

sudo ip ad add 10.0.0.1/24 dev eth0
sudo ip link set dev eth0 up
export ROS_HOSTNAME=10.0.0.1
export ROS_MASTER_URI=http://10.0.0.1:11311

# 		MTR 	K(BCM)	W(BCM)	Y(BCM) B(BCM) 	KP 	KI 	KD 	ErrThrs	LR Verbose
export MotorLW="LW	15	14	18	23	1.5	0.001	2	5	40 1"
export MotorRH="RH	25	24	12	16	2.0	0.001	2	5	40 0"
export MotorLH="LH	17	27	20	21	2.0	0.002	2	5	40 0"
export MotorRW="RW	6	5	19	26	1.5	0.001	2	5	40 1"
# 		Instructions	WRISTAW H{OC} 	TIMEOUT OFFW{L,R,F,B}	PTIME	CAL{DST}
export master="	F		540	0 660	1.00	90 90 90 90	0.9	100 1.0 50"
#bdLFt
#BfTlRF -- 13 sec
#tBDflRbrd -- 30 sec
#lRTbdLFL -- 30 sec
#tfTrfbDl -- 36 sec

roslaunch rubiker run.launch

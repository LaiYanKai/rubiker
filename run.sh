#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

sudo ip addr add 10.0.0.1/24 dev eth0
sudo ip link set dev eth0 up

# 		MTR 	K(BCM)	W(BCM)	Y(BCM) 	B(BCM) 	KP 	KI 	KD 	ErrThrs	LR
#export MotorLW="LW	15	14	23	18	1.5	0.001	2	5	40 1" #M5 #+ACW -CW
#export MotorRH="RH	25	24	16	12	2.0	0.001	2	5	40 1" #M2 #+Close -Open #may have loose K and W wires
#export MotorLH="LH	6	5	26	19	2.0	0.002	2	5	40 1" #M4 #+Close -Open
export MotorRW="RW	17	27	21	20	1.5	0.001	2	5	40 1" #M1 
#		Instructions	WRISTAW	H{OC}	TIMEOUT	OFFW{LRFB}	PTIME	CAL{DST}
export master="	F		270	0 330	1.00	90 90 90 90	0.9	100 50 1.0

#bdLFt
#BfTlRF -- 13 s
#tBDflRbrd -- 30s
#lRTbdLFL -- 30s
#tfTrfbDl -- 36s

roslaunch rubiker run.launch

#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

#sudo ip ad add 10.0.0.1/24 dev eth0

# 		MTR 	K(BCM)	W(BCM)	Y(BCM) B(BCM) 	KP 	KI 	KD 	ErrThrs	LR
export Motor="LW	15	14	18	23	1.5	0.001	2	5	40 1"
#export Motor="RH	25	24	12	16	2.0	0.001	2	5	40 0"
#export Motor="LH	17	27	20	21	2.0	0.002	2	5	40 0"
#export Motor="RW	6	5	19	26	1.5	0.001	2	5	40 1"

roslaunch rubiker test.launch

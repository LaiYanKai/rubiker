#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"

#sudo ip ad add 10.0.0.1/24 dev eth0

# 		MTR 	K(BCM)	W(BCM)	Y(BCM) 	B(BCM) 	KP 	KI 	KD 	ErrThrs	LR
#export Motor="LW	15	14	23	18	1.5	0.001	2	5	40 1" #M5 #+ACW -CW
#export Motor="RH	25	24	16	12	2.0	0.001	2	5	40 1" #M2 #+Close -Open #may have loose K and W wires
#export Motor="LH	6	5	26	19	2.0	0.002	2	5	40 1" #M4 #+Close -Open
export Motor="RW	17	27	21	20	1.5	0.001	2	5	40 1" #M1 
roslaunch rubiker test.launch

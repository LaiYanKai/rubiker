#PARAMETERS
source devel/setup.bash
echo "BASH SOURCED"


# MTR PIN_K(BCM) PIN_W(BCM) PIN_Y(WPI/Name) PIN_B(WPI/Name) KP KI KD
export M1="1 14 15 1 4 0.4 0.05 0.35"
export M2="2 8 7 5 6 0.4 0.05 0.35"
export M3="3 1 12 27 28 0.4 0.05 0.35"
export M4="4 27 22 12 13 0.4 0.05 0.35"
# export M4="4 6 13 24 25 0.5 0.05 0.6"
roslaunch rubiker rubiker.launch

echo "================= RESETING ===================="
source reset.sh

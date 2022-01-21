source devel/setup.bash
chmod +x src/rubiker/scripts/*.py

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# 		Gamma	Coords
export Vision="	0.6	95,90 155,90 220,90 90,150 155,150 225,150 85,210 155,210 230,210"

roslaunch rubiker vision.launch

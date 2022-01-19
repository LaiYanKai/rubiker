source devel/setup.bash
chmod +x src/rubiker/scripts/*.py

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# 		Gamma	Coords
export Vision="	0.6	90,65 155,65 220,65 85,125 155,125 225,125 80,185 155,185 230,185"

roslaunch rubiker test_vision.launch

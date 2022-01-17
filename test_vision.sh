source devel/setup.bash
chmod +x src/rubiker/scripts/*.py

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# 		Gamma	Coords
export Vision="	0.6	95,70 150,70 200,70 90,125 150,125 205,125 90,180 150,180 210,180"

roslaunch rubiker test_vision.launch

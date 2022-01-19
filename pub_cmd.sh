source devel/setup.bash

rostopic pub -1 /cmdRH rubiker/MotorCmd "{seq:0, type:116, target: -540, relative:false, speed: -30, duration 2.0, end 104}"

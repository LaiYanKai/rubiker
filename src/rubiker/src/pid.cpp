#include <ros/ros.h>
#include <softPwm.h>
#include <std_msgs/Int32.h>
#include <wiringPi.h>
#include <iostream>
#include <string>
#include <ros/console.h>
#include  <std_msgs/Bool.h>

//#define PIN_K 23 // change pin number here
//#define PIN_W 24
//#define KP 6.0
//#define KI 0.05
//#define KD 6.0

/*
void callback(const std_msgs::Int32::ConstPtr& target, const std_msgs::Int32::ConstPtr& deg) {
    ROS_INFO("Target: [%d], Degrees: [%d], Error: [%d]", target->data, deg->data, (target->data - deg->data));
}
*/

int target = 0;
int deg = 0;
bool stop = false;
void cbTarget(const std_msgs::Int32::ConstPtr& msg)
{
    target = msg->data;
//    ROS_INFO("tgt[%d]", target);
}
void cbDeg(const std_msgs::Int32::ConstPtr& msg)
{
    deg = msg->data;
//    ROS_INFO("[%d]", deg);
}
void cbStop(const std_msgs::Bool::ConstPtr& msg)
{
    stop = msg->data;
}
float sign(float x) {
    if (x > 0) {
	return 1;
    } else if (x < 0) {
	return -1;
    } else {
	return 0;
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pid");
    ros::NodeHandle nh;

    std::string MTR(argv[1]);
    std::string TOPIC_TARGET = "target" + MTR;
    std::string TOPIC_DEG = "deg" + MTR;
    int PIN_K = strtol(argv[2], nullptr, 0);
    int PIN_W = strtol(argv[3], nullptr, 0);

    float KP = strtof(argv[6], nullptr);
    float KI = strtof(argv[7], nullptr);
    float KD = strtof(argv[8], nullptr);
    ROS_INFO("%8s --> PIN_K:%02d  PIN_W:%02d  KP:%9.5f  KI:%9.5f  KD:%9.5f", ros::this_node::getName().c_str(), PIN_K, PIN_W, KP, KI, KD);

    wiringPiSetupGpio();

    softPwmCreate(PIN_K, 0, 100);
    softPwmCreate(PIN_W, 0, 100);

    ros::Rate r(40);

    ros::Subscriber sub_target = nh.subscribe(TOPIC_TARGET, 1, cbTarget);
    ros::Subscriber sub_deg = nh.subscribe(TOPIC_DEG, 1, cbDeg);
    ros::Subscriber sub_stop = nh.subscribe("stop", 1, cbStop);

    float accum = 0;
    float prev_error = 0;
    float error, prop, integ, deriv, res;
    int res_int;

    while (ros::ok() && !stop)
    {

        ros::spinOnce();

	error = target - deg;
        prop = KP * error;
        integ = KI * (error-prev_error);
	deriv = KD * (error - prev_error);
	res = prop + integ + deriv;
	res_int = (int) res;
	
	// Anti wind-up control
	if (sign(error) != sign(prev_error)) {
	    accum = 0;
	}

	if (res > 0) {
	    if (res_int > 100)
		res_int = 100;
	    softPwmWrite(PIN_K, 0);
	    softPwmWrite(PIN_W, res_int);
	} else {
            if (res_int < -100)
		res_int = -100;
	    softPwmWrite(PIN_K, -res_int);
	    softPwmWrite(PIN_W, 0);
	}

	prev_error = error;
        r.sleep();
    }

    softPwmWrite(PIN_K, 0);
    softPwmWrite(PIN_W, 0);
    ros::spin();
    return 0;
}

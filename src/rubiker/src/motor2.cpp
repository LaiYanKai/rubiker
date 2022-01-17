#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#define MAX_RESULT 100

int ACCEPTABLE_ERROR = 5;
int PIN_Y = 0;
int PIN_B = 0;
int deg = 0;
int target = 0;

bool state_a = false;
bool state_b = false;
bool changed_deg = true;
bool changed_target = false;
bool stop = false;
bool verbose = false;
bool timeout_received = false;

void tacho(bool tac)
{
   // bool dir = state_a == ((!state_b && tac) || (state_b && !tac));
    bool dir = state_a == (state_b != tac);
   // TO DO
    if(dir){
	deg++;
    }else{
	deg--;
    }
    changed_deg = true;
}
void handler_a(void)
{
    tacho(1);
    state_a = digitalRead(PIN_Y);
//    ROS_INFO_STREAM("a called to: " << state_a);
}
void handler_b(void)
{
    tacho(0);
    state_b = digitalRead(PIN_B);
//    ROS_INFO_STREAM("b called to: " << state_b);
}

float sign(float x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
}
void cbTarget(const std_msgs::Int32::ConstPtr& msg)
{
    target = msg->data;
    changed_target = true;
}
void cbStop(const std_msgs::Bool::ConstPtr& msg)
{
    stop = msg->data;
}
void cbTimeout(const std_msgs::Empty::ConstPtr& msg)
{
    timeout_received = true;
}
int main (int argc, char **argv)
{
    ros::init(argc, argv, "motor");
    ros::NodeHandle nh;

    // Parse arguments
    std::string MTR(argv[1]);
    std::string TOPIC_TARGET = "target" + MTR;
    std::string TOPIC_TIMEOUT = "timeout" + MTR;
    std::string TOPIC_COMP = "comp" + MTR;
    int PIN_K = strtol(argv[2], nullptr, 0);
    int PIN_W = strtol(argv[3], nullptr, 0);
    PIN_Y = strtol(argv[4], nullptr, 0);
    PIN_B = strtol(argv[5], nullptr, 0);
    float KP = strtof(argv[6], nullptr);
    float KI = strtof(argv[7], nullptr);
    float KD = strtof(argv[8], nullptr);
    ACCEPTABLE_ERROR = strtol(argv[9], nullptr, 0);
    verbose = strtol(argv[10], nullptr, 0);
    ROS_INFO("%2s --> K:%02d  W:%02d  Y:%02d  B:%02d  KP:%9.5f  KI:%9.5f  KD:%9.5f  Verbose:%d", MTR.c_str(), PIN_K, PIN_W, PIN_Y, PIN_B, KP, KI, KD, verbose);

    // Prepare pins with WiringPi
    if (wiringPiSetupGpio() >  0) { return 1; }
    pinMode(PIN_Y, INPUT);
    pinMode(PIN_B, INPUT);
    state_a = digitalRead(PIN_Y);
    state_b = digitalRead(PIN_B);
    if (wiringPiISR(PIN_Y, INT_EDGE_BOTH, &handler_a)  < 0) { ROS_INFO("loser1"); return 1; }
    if (wiringPiISR(PIN_B, INT_EDGE_BOTH, &handler_b)  < 0) { ROS_INFO("loser1"); return 1; }
    softPwmCreate(PIN_K, 0, 100);
    softPwmCreate(PIN_W, 0, 100);

    // subscribers
    ros::Subscriber sub_target = nh.subscribe(TOPIC_TARGET, 1, cbTarget);
    ros::Subscriber sub_stop = nh.subscribe("stop", 1, cbStop);
    ros::Subscriber sub_timeout = nh.subscribe(TOPIC_TIMEOUT, 1, cbTimeout);
    ros::Publisher pub_comp = nh.advertise<std_msgs::Empty>(TOPIC_COMP, 1, true);
    std_msgs::Empty msg_comp;

    // Prepare PID variables
    float accum = 0;
    float prev_error = 0;
    float error,  prop, integ, deriv, result;
    int result_int;
    ros::Rate looper(40);
    int prev_exec_counter = -1;

    pub_comp.publish(msg_comp);
    // Main Loop
    while (ros::ok() && !stop)
    {
        // verbose for testing
        if (verbose && changed_deg) {
            ROS_INFO_STREAM("deg" << MTR << ": " << deg);
            changed_deg = false;
        }

        // Error
        error = target - deg;

        if (timeout_received)
        {
            timeout_received = false;
            target = deg;
            error = 0;
            ROS_INFO("%2s: Timeout Received", MTR.c_str());
        }

        if (changed_target && abs(error) < ACCEPTABLE_ERROR)
        {
            pub_comp.publish(msg_comp);
            changed_target = false;
        }


        // PID
        prop = KP * error;
        accum += error;
        integ = KI * accum;
        deriv = KD * (error - prev_error);
        prev_error = error;
        result = prop + integ + deriv;
        result_int = (int) result;

        if (sign(error) != sign(prev_error))
            accum = 0;

        if (result >= 0)
        {
            if (result_int > MAX_RESULT)
                result_int = MAX_RESULT;
            softPwmWrite(PIN_K, 0);
            softPwmWrite(PIN_W, result_int);
        }
        else
        {
            if (result_int < -MAX_RESULT)
                result_int = -MAX_RESULT;
            softPwmWrite(PIN_K, -result_int);
            softPwmWrite(PIN_W, 0);
        }


        ros::spinOnce();
	looper.sleep();
    }

    softPwmWrite(PIN_K, 0);
    softPwmWrite(PIN_W, 0);
    return 0;
}


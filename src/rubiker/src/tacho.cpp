#include <ros/ros.h>
#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"

int PIN_Y = 0;
int PIN_B = 0;
int deg = 0;

bool state_a = false;
bool state_b = false;

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


int main (int argc, char **argv)
{
    ros::init(argc, argv, "tacho");
    ros::NodeHandle nh;
    
    int MTR = strtol(argv[1], nullptr, 0);
    std::string TOPIC_DEG = "deg" + std::to_string(MTR);
    PIN_Y = strtol(argv[4], nullptr, 0);
    PIN_B = strtol(argv[5], nullptr, 0);
    ROS_INFO("%8s (%02d) --> PIN_Y:%02d  PIN_B:%02d", ros::this_node::getName().c_str(), MTR, PIN_Y, PIN_B);

    // publish the interrupt (deg) into topic / deg
    ros::Publisher tacho_topic =  nh.advertise<std_msgs::Int32>(TOPIC_DEG, 1, true);
    if (wiringPiSetup() >  0) { return 1; }
    pinMode(PIN_Y, INPUT);
    pinMode(PIN_B, INPUT);
    state_a = digitalRead(PIN_Y);
    state_b = digitalRead(PIN_B);
//    ROS_INFO_STREAM ("init " <<  state_a << " " << state_b << " " << PIN_Y << " " << PIN_B);
    if (wiringPiISR(PIN_Y, INT_EDGE_BOTH, &handler_a)  < 0) { ROS_INFO("loser1"); return 1; }
    if (wiringPiISR(PIN_B, INT_EDGE_BOTH, &handler_b)  < 0) { ROS_INFO("loser1"); return 1; }

    std_msgs::Int32 msg;
    ros::Rate looper(10);
    while (ros::ok())
    {
	msg.data =  deg;
	tacho_topic.publish(msg);
//        ROS_INFO_STREAM(digitalRead(PIN_Y) <<" ,"<< digitalRead(PIN_B) << " ," << digitalRead(1) << "," << digitalRead(4));

        ros::spinOnce();
	looper.sleep();
    }

}


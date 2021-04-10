#include <ros/ros.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "reset");
    ros::NodeHandle nm;
    wiringPiSetupGpio();

    std::vector<int> pins = {14, 6, 15, 13, 7, 12}; // pins to set 30 % for 3 seconds
    std::vector<int> pins_pwm = {1, 8}; // pins to set digital Low // 1, 12, 8, 7

    for (auto i : pins)
    {
       pinMode(i, OUTPUT);
       digitalWrite(i, 0);
    }

    for (auto i : pins_pwm)
    {
       softPwmCreate(i, 0, 100);
       softPwmWrite(i, 20);
    }

    ros::Duration(5).sleep();

    for (auto i : pins_pwm)
    {
       softPwmWrite(i, 0);
    }


//14 15
// 6 13
// 2 8 7
// 3 1 12

//    ros::Publisher pub_target1 = nm.advertise<std_msgs::Int32>("target1", 1, true);
//    ros::Publisher pub_target2 = nm.advertise<std_msgs::Int32>("target2", 1, true);
//    ros::Publisher pub_target3 = nm.advertise<std_msgs::Int32>("target3", 1, true);
//    ros::Publisher pub_target4 = nm.advertise<std_msgs::Int32>("target4", 1, true);
//    ros::Subscriber sub_deg1 = nm.subscribe("deg1", 1, cbDeg1);
//    ros::Subscriber sub_deg2 = nm.subscribe("deg2", 1, cbDeg2);
//    ros::Subscriber sub_deg3 = nm.subscribe("deg3", 1, cbDeg3);
//    ros::Subscriber sub_deg4 = nm.subscribe("deg4", 1, cbDeg4);


//    ros::Rate looper(10);

//    msg.data = targets1[0];
//    pub_target1.publish(msg);
//    msg.data = targets2[0];
//    pub_target2.publish(msg);
//    msg.data = targets3[0];
//    pub_target3.publish(msg);
//    msg.data = targets4[0];
//    pub_target4.publish(msg);
//    ros::spinOnce();
    return 0;
}

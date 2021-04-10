#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"

#define THRESH 3
int deg1 = 0;
int deg2 = 0;
int deg3 = 0;
int deg4 = 0;

void cbDeg1(const std_msgs::Int32::ConstPtr& msg)
{
   deg1 = msg->data;
}
void cbDeg2(const std_msgs::Int32::ConstPtr& msg)
{
   deg2 = msg->data;
}
void cbDeg3(const std_msgs::Int32::ConstPtr& msg)
{
   deg3 = msg->data;
}
void cbDeg4(const std_msgs::Int32::ConstPtr& msg)
{
   deg4 = msg->data;
}

// W1C3
// W4C2
int main(int argc, char **argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nm;

    ros::Publisher pub_target1 = nm.advertise<std_msgs::Int32>("target1", 1, true);
    ros::Publisher pub_target2 = nm.advertise<std_msgs::Int32>("target2", 1, true);
    ros::Publisher pub_target3 = nm.advertise<std_msgs::Int32>("target3", 1, true);
    ros::Publisher pub_target4 = nm.advertise<std_msgs::Int32>("target4", 1, true);
    ros::Subscriber sub_deg1 = nm.subscribe("deg1", 1, cbDeg1);
    ros::Subscriber sub_deg2 = nm.subscribe("deg2", 1, cbDeg2);
    ros::Subscriber sub_deg3 = nm.subscribe("deg3", 1, cbDeg3);
    ros::Subscriber sub_deg4 = nm.subscribe("deg4", 1, cbDeg4);


    ros::Rate looper(10);

    std_msgs::Int32 msg;
    std::vector<int> targets1 = {-540}; // Wrist 
    std::vector<int> targets2 = {720}; // Claw  1/6 {180, -180, 360, -360};
    std::vector<int> targets3 = {720}; // Claw 1/6 {180, -180, 360, -360};
    std::vector<int> targets4 = {-540}; // Wrist {180, -180, 360, -360};

    msg.data = targets1[0];
    pub_target1.publish(msg);
    msg.data = targets2[0];
    pub_target2.publish(msg);
    msg.data = targets3[0];
    pub_target3.publish(msg);
    msg.data = targets4[0];
    pub_target4.publish(msg);
    ros::spinOnce();

    bool done1 = false;
    bool done2 = false;
    bool done3 = false;
    bool done4 = false;

    int i1 = 0, i2 = 0, i3 = 0, i4 = 0;
    while (ros::ok() && !(done1 && done2 && done3 && done4)){
        if (!done1 && abs(deg1 - targets1[i1]) < THRESH){
            i1++;
            if (targets1.size() <= i1){
                done1 = true;
                ROS_INFO_STREAM("1 reached");
            } else {
                msg.data = targets1[i1];
                pub_target1.publish(msg);
                ROS_INFO_STREAM("1 Changed Target");
            }
        }
        if (!done2 && abs(deg2 - targets2[i2]) < THRESH){
            i2++;
            if (targets2.size() <= i2){
                done2 = true;
                ROS_INFO_STREAM("2 Reached");
            } else {
                msg.data = targets2[i2];
                pub_target2.publish(msg);
                ROS_INFO_STREAM("2 Changed Target");
            }
        }
        if (!done3 && abs(deg3 - targets3[i3]) < THRESH){
            i3++;
            if (targets3.size() <= i3){
                done3 = true;
                ROS_INFO_STREAM("3 Reached");
            } else {
                msg.data = targets3[i3];
                pub_target3.publish(msg);
                ROS_INFO_STREAM("3 Changed Target");
            }
        }
        if (!done4 && abs(deg4 - targets4[i4]) < THRESH){
            i4++;
            if (targets4.size() <= i4){
                done4 = true;
                ROS_INFO_STREAM("4 Reached");
            } else {
                msg.data = targets4[i4];
                pub_target4.publish(msg);
                ROS_INFO_STREAM("4 Changed Target");
            }
        }
        looper.sleep();
        ros::spinOnce();
    }



}

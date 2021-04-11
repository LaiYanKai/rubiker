#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"

#define THRESH 3
#define WRISTCW 720
#define WRISTACW -720
#define HANDOPEN 540
#define HANDCLOSE 720
#define L_CW 1
#define L_ACW 2
#define L_CLOSE 3
#define L_OPEN 4
#define R_CW 5
#define R_ACW 6
#define R_CLOSE 7
#define R_OPEN 8
#define F_CW 9
#define F_ACW 10
#define F_CLOSE 11
#define F_OPEN 12
#define B_CW 13
#define B_ACW 14
#define B_CLOSE 15
#define B_OPEN 16
#define L_TURNOPEN 17
#define R_TURNOPEN 18
#define F_TURNOPEN 19
#define B_TURNOPEN 20
#define L_CW_R_ACW 21

int degLW = 0;
int degRH = 0;
int degLH = 0;
int degRW = 0;
int degFW = 0;
int degBH = 0;
int degFH = 0;
int degBW = 0;

void cbDegLW(const std_msgs::Int32::ConstPtr& msg)
{
   degLW = msg->data;
}
void cbDegRH(const std_msgs::Int32::ConstPtr& msg)
{
   degRH = msg->data;
}
void cbDegLH(const std_msgs::Int32::ConstPtr& msg)
{
   degLH = msg->data;
}
void cbDegRW(const std_msgs::Int32::ConstPtr& msg)
{
   degRW = msg->data;
}
void cbDegFW(const std_msgs::Int32::ConstPtr& msg)
{
   degFW = msg->data;
}
void cbDegBH(const std_msgs::Int32::ConstPtr& msg)
{
   degBH = msg->data;
}
void cbDegFH(const std_msgs::Int32::ConstPtr& msg)
{
   degFH = msg->data;
}
void cbDegBW(const std_msgs::Int32::ConstPtr& msg)
{
   degBW = msg->data;
}

// W1C3
// W4C2
int main(int argc, char **argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nm;

    ros::Publisher pub_target1 = nm.advertise<std_msgs::Int32>("targetLW", 1, true);
    ros::Publisher pub_target2 = nm.advertise<std_msgs::Int32>("targetRH", 1, true);
    ros::Publisher pub_target3 = nm.advertise<std_msgs::Int32>("targetLH", 1, true);
    ros::Publisher pub_target4 = nm.advertise<std_msgs::Int32>("targetRW", 1, true);
    ros::Publisher pub_target1 = nm.advertise<std_msgs::Int32>("targetFW", 1, true);
    ros::Publisher pub_target2 = nm.advertise<std_msgs::Int32>("targetBH", 1, true);
    ros::Publisher pub_target3 = nm.advertise<std_msgs::Int32>("targetFH", 1, true);
    ros::Publisher pub_target4 = nm.advertise<std_msgs::Int32>("targetBW", 1, true);
    ros::Subscriber sub_deg1 = nm.subscribe("degLW", 1, cbDegLW);
    ros::Subscriber sub_deg2 = nm.subscribe("degRH", 1, cbDegRH);
    ros::Subscriber sub_deg3 = nm.subscribe("degLH", 1, cbDegLH);
    ros::Subscriber sub_deg4 = nm.subscribe("degRW", 1, cbDegRW);
    ros::Subscriber sub_deg1 = nm.subscribe("degFW", 1, cbDegFW);
    ros::Subscriber sub_deg2 = nm.subscribe("degBH", 1, cbDegBH);
    ros::Subscriber sub_deg3 = nm.subscribe("degFH", 1, cbDegFH);
    ros::Subscriber sub_deg4 = nm.subscribe("degBW", 1, cbDegBW);


    ros::Rate looper(10);

    std_msgs::Int32 msg;
    bool upL = true, upR = true, upF = true, upB = true,
         cL = true, cR = true, cF = true, cB = true, rotated = false;
    std::vector<char>instructions = {'L','l','R','r','F','f','B','b','T','t','D','d'}
    std::vector<int>states = {};
    for(auto instruction: instructions)
    {
        if(instruction == 'L'){
            if(rotated){
                if(upF){
                    if(!cF){
                        states.push_back(F_CLOSE);
                        cF = !cF;
                    }
                }else{
                    if(!cF){
                        states.push_back(F_CW);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                        cF = !cF;
                    }else{
                        states.push_back(F_TURNOPEN);
                        cF = !cF;
                        upF = !upF;
                        states.push_back(F_CLOSE);
                        cF = !cF;
                    }
                }
                if(upB){
                    if(!cB){
                        states.push_back(B_CLOSE);
                        cB = !cB;
                    }
                }else{
                    if(!cB){
                        states.push_back(B_CW);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                        cB = !cB;
                    }else{
                        states.push_back(B_TURNOPEN);
                        cB = !cB;
                        upB = !upB;
                        states.push_back(B_CLOSE);
                        cB = !cB;
                    }
                }if(!upR){
                    if(!cR){
                        states.push_back(R_CLOSE);
                        cR = !cR;
                    }
                }else{
                    if(!cR){
                        states.push_back(R_CW);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                        cR = !cR;
                    }else{
                        states.push_back(R_TURNOPEN);
                        cR = !cR;
                        upR = !upR;
                        states.push_back(R_CLOSE);
                        cR = !cR;
                }if(!upL){
                    if(!cL){
                        states.push_back(L_CLOSE);
                        cL = !cL;
                    }
                }else{
                    if(!cL){
                        states.push_back(L_CW);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                        cL = !cL;
                    }else{
                        states.push_back(L_TURNOPEN);
                        cL = !cL;
                        upL = !upL;
                        states.push_back(L_CLOSE);
                        cL = !cL;
                     }
                  states.push_back(F_OPEN);
                  cF = !cF;
                  states.push_back(B_OPEN);
                  cB = !cB;
                  states.push_back(L_CW_R_ACW);
                  upL = !upL;
                  upR = !upR;
                  xxx
            }
        } else if (instruction == 'l'){
        } else if (instruction == 'R'){
        } else if (instruction == 'r'){
        } else if (instruction == 'F'){
        } else if (instruction == 'f'){
        } else if (instruction == 'B'){
        } else if (instruction == 'b'){
        } else if (instruction == 'T'){
        } else if (instruction == 't'){
        } else if (instruction == 'D'){
        } else if (instruction == 'd'){
        }
    }

    while(ros::ok())
    {
        looper.sleep();
        ros::spinOnce();
    }
/*
    std::vector<int> targets1 = {140}; // Wrist
    std::vector<int> targets2 = {0}; // Claw  1/6 {180, -180, 360, -360};
    std::vector<int> targets3 = {0}; // Claw 1/6 {180, -180, 360, -360};
    std::vector<int> targets4 = {0}; // Wrist {180, -180, 360, -360};

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


*/
}

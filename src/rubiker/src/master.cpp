#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <wiringPi.h>
#include <softPwm.h>

#define THRESH 15
#define WRISTCW -540
#define WRISTACW 540
#define HANDOPEN -450
#define HANDCLOSE 0
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
#define L_ACW_R_CW 22
#define TIMEOUT 0.75

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
    wiringPiSetupGpio();

    ros::Publisher pub_targetLW = nm.advertise<std_msgs::Int32>("targetLW", 1, true);
    ros::Publisher pub_targetRH = nm.advertise<std_msgs::Int32>("targetRH", 1, true);
    ros::Publisher pub_targetLH = nm.advertise<std_msgs::Int32>("targetLH", 1, true);
    ros::Publisher pub_targetRW = nm.advertise<std_msgs::Int32>("targetRW", 1, true);
    ros::Publisher pub_targetFW = nm.advertise<std_msgs::Int32>("targetFW", 1, true);
    ros::Publisher pub_targetBH = nm.advertise<std_msgs::Int32>("targetBH", 1, true);
    ros::Publisher pub_targetFH = nm.advertise<std_msgs::Int32>("targetFH", 1, true);
    ros::Publisher pub_targetBW = nm.advertise<std_msgs::Int32>("targetBW", 1, true);
    ros::Publisher pub_stop = nm.advertise<std_msgs::Bool>("stop", 1, true);
    ros::Subscriber sub_degLW = nm.subscribe("degLW", 1, cbDegLW);
    ros::Subscriber sub_degRH = nm.subscribe("degRH", 1, cbDegRH);
    ros::Subscriber sub_degLH = nm.subscribe("degLH", 1, cbDegLH);
    ros::Subscriber sub_degRW = nm.subscribe("degRW", 1, cbDegRW);
    ros::Subscriber sub_degFW = nm.subscribe("degFW", 1, cbDegFW);
    ros::Subscriber sub_degBH = nm.subscribe("degBH", 1, cbDegBH);
    ros::Subscriber sub_degFH = nm.subscribe("degFH", 1, cbDegFH);
    ros::Subscriber sub_degBW = nm.subscribe("degBW", 1, cbDegBW);
    int targetLH = 0, targetLW = 0, targetRH = 0, targetRW = 0,
        targetBH = 0, targetBW = 0, targetFH = 0, targetFW = 0;

    ros::Rate looper(50);

    std_msgs::Int32 msg;
    std_msgs::Bool msg_stop;
    msg_stop.data = false;
    pub_stop.publish(msg_stop);

    bool upL = true, upR = true, upF = true, upB = true,
         cL = true, cR = true, cF = true, cB = true, rotated = false;
    std::string instructions(argv[1]); //{'L','l','R','r','F','f','B','b','T','t','D','d'};
    std::vector<int>states = {};
    for(auto instruction: instructions)
    {
        ROS_INFO_STREAM("Instruction: " << instruction);
        if(instruction == 'L'){ 
            if(!rotated){
                if(!upL){
                    states.push_back(L_CW);
                    upL = !upL;
                    ROS_INFO_STREAM("L_CW");
                }else if(upF && upB){
                    states.push_back(L_CW);
                    upL = !upL;
                    ROS_INFO_STREAM("L_CW");
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                        ROS_INFO_STREAM("F_TURNOPEN F_CLOSE");
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                        ROS_INFO_STREAM("B_TURNOPEN B_CLOSE");
                    }
                    states.push_back(L_CW);
                    upL = !upL;
                    ROS_INFO_STREAM("L_CW");
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                    ROS_INFO_STREAM("F_TURNOPEN");
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                    ROS_INFO_STREAM("B_TURNOPEN");
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                ROS_INFO_STREAM("L_ACW_R_CW");
                if(!upL){
                    states.push_back(L_CW);
                    upL = !upL;
                    ROS_INFO_STREAM("L_CW");
                }else if(upF && upB){
                    states.push_back(L_CW);
                    upL = !upL;
                    ROS_INFO_STREAM("L_CW");
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                        ROS_INFO_STREAM("F_TURNOPEN F_CLOSE");
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                        ROS_INFO_STREAM("B_TURNOPEN B_CLOSE");
                    }
                    states.push_back(L_CW);
                    upL = !upL;
                    ROS_INFO_STREAM("L_CW");
                }
            }
        } else if (instruction == 'l'){
            if(!rotated){
                if(!upL){
                    states.push_back(L_ACW);
                    upL = !upL;
                }else if(upF && upB){
                    states.push_back(L_ACW);
                    upL = !upL;
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                    }
                    states.push_back(L_ACW);
                    upL = !upL;
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                if(!upL){
                    states.push_back(L_ACW);
                    upL = !upL;
                }else if(upF && upB){
                    states.push_back(L_ACW);
                    upL = !upL;
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                    }
                    states.push_back(L_ACW);
                    upL = !upL;
                }
            }

        } else if (instruction == 'R'){
            if(!rotated){
                if(!upR){
                    states.push_back(R_CW);
                    upR = !upR;
                }else if(upF && upB){
                    states.push_back(R_CW);
                    upR = !upR;
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                    }
                    states.push_back(R_CW);
                    upR = !upR;
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                if(!upR){
                    states.push_back(R_CW);
                    upR = !upR;
                }else if(upF && upB){
                    states.push_back(R_CW);
                    upR = !upR;
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                    }
                    states.push_back(R_CW);
                    upR = !upR;
                }

            }

        } else if (instruction == 'r'){
            if(!rotated){
                if(!upR){
                    states.push_back(R_ACW);
                    upR = !upR;
                }else if(upF && upB){
                    states.push_back(R_ACW);
                    upR = !upR;
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                    }
                    states.push_back(R_ACW);
                    upR = !upR;
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                if(!upR){
                    states.push_back(R_ACW);
                    upR = !upR;
                }else if(upF && upB){
                    states.push_back(R_ACW);
                    upR = !upR;
                }else{
                    if(!upF){
                        states.push_back(F_TURNOPEN);
                        upF = !upF;
                        states.push_back(F_CLOSE);
                    }
                    if(!upB){
                        states.push_back(B_TURNOPEN);
                        upB = !upB;
                        states.push_back(B_CLOSE);
                    }
                    states.push_back(R_ACW);
                    upR = !upR;
                }
            }
        } else if (instruction == 'F'){
            if(!rotated){
                if(!upF){
                    states.push_back(F_CW);
                    upF = !upF;
                }else if(upR && upL){
                    states.push_back(F_CW);
                    upF = !upF;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(F_CW);
                    upF = !upF;
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                if(!upF){
                    states.push_back(F_CW);
                    upF = !upF;
                }else if(upR && upL){
                    states.push_back(F_CW);
                    upF = !upF;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(F_CW);
                    upF = !upF;
                }
            }
        } else if (instruction == 'f'){
            if(!rotated){
                if(!upF){
                    states.push_back(F_ACW);
                    upF = !upF;
                }else if(upR && upL){
                    states.push_back(F_ACW);
                    upF = !upF;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(F_ACW);
                    upF = !upF;
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                if(!upF){
                    states.push_back(F_ACW);
                    upF = !upF;
                }else if(upR && upL){
                    states.push_back(F_ACW);
                    upF = !upF;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(F_ACW);
                    upF = !upF;
                }
            }
        } else if (instruction == 'B'){
            if(!rotated){
                if(!upB){
                    states.push_back(B_CW);
                    upB = !upB;
                }else if(upR && upL){
                    states.push_back(B_CW);
                    upB = !upB;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(B_CW);
                    upB = !upB;
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                if(!upB){
                    states.push_back(B_CW);
                    upB = !upB;
                }else if(upR && upL){
                    states.push_back(B_CW);
                    upB = !upB;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(B_CW);
                    upB = !upB;
                }
            }
        } else if (instruction == 'b'){
            if(!rotated){
                if(!upB){
                    states.push_back(B_ACW);
                    upB = !upB;
                }else if(upR && upL){
                    states.push_back(B_ACW);
                    upB = !upB;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(B_ACW);
                    upB = !upB;
                }
            }else if(rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                }
                if(upB){
                    states.push_back(B_OPEN);
                }
                if(upF){
                    states.push_back(F_OPEN);
                }
                states.push_back(L_ACW_R_CW);
                upL = !upL;
                upR = !upR;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                rotated = !rotated;
                if(!upB){
                    states.push_back(B_ACW);
                    upB = !upB;
                }else if(upR && upL){
                    states.push_back(B_ACW);
                    upB = !upB;
                }else{
                    if(!upR){
                        states.push_back(R_TURNOPEN);
                        upR = !upR;
                        states.push_back(R_CLOSE);
                    }
                    if(!upL){
                        states.push_back(L_TURNOPEN);
                        upL = !upL;
                        states.push_back(L_CLOSE);
                    }
                    states.push_back(B_ACW);
                    upB = !upB;
                }
            }
        } else if (instruction == 'T'){
            if(rotated){
                states.push_back(F_CW);
                upF = !upF;
            }else if (!rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                    states.push_back(F_CLOSE);
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                    states.push_back(B_CLOSE);
                }
                if(upL){
                    states.push_back(L_TURNOPEN);
                    upL = !upL;
                    states.push_back(L_CLOSE);
                }
                if(upR){
                    states.push_back(R_TURNOPEN);
                    upR = !upR;
                    states.push_back(R_CLOSE);
                }
                states.push_back(F_OPEN);
                states.push_back(B_OPEN);
                states.push_back(L_CW_R_ACW);
                upL = !upL;
                upR = !upR;
                rotated = !rotated;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                states.push_back(F_CW);
                upF = !upF;
            }
        } else if (instruction == 't'){
            if(rotated){
                states.push_back(F_ACW);
                upF = !upF;
            }else if (!rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                    states.push_back(F_CLOSE);
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                    states.push_back(B_CLOSE);
                }
                if(upL){
                    states.push_back(L_TURNOPEN);
                    upL = !upL;
                    states.push_back(L_CLOSE);
                }
                if(upR){
                    states.push_back(R_TURNOPEN);
                    upR = !upR;
                    states.push_back(R_CLOSE);
                }
                states.push_back(F_OPEN);
                states.push_back(B_OPEN);
                states.push_back(L_CW_R_ACW);
                upL = !upL;
                upR = !upR;
                rotated = !rotated;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                states.push_back(F_ACW);
                upF = !upF;
            }
        } else if (instruction == 'D'){
            if(rotated){
                states.push_back(B_CW);
                upB = !upB;
            }else if (!rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                    states.push_back(F_CLOSE);
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                    states.push_back(B_CLOSE);
                }
                if(upL){
                    states.push_back(L_TURNOPEN);
                    upL = !upL;
                    states.push_back(L_CLOSE);
                }
                if(upR){
                    states.push_back(R_TURNOPEN);
                    upR = !upR;
                    states.push_back(R_CLOSE);
                }
                states.push_back(F_OPEN);
                states.push_back(B_OPEN);
                states.push_back(L_CW_R_ACW);
                upL = !upL;
                upR = !upR;
                rotated = !rotated;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                states.push_back(B_CW);
                upB = !upB;
            }
        } else if (instruction == 'd'){
            if(rotated){
                states.push_back(B_ACW);
                upB = !upB;
            }else if (!rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    upF = !upF;
                    states.push_back(F_CLOSE);
                }
                if(!upB){
                    states.push_back(B_TURNOPEN);
                    upB = !upB;
                    states.push_back(B_CLOSE);
                }
                if(upL){
                    states.push_back(L_TURNOPEN);
                    upL = !upL;
                    states.push_back(L_CLOSE);
                }
                if(upR){
                    states.push_back(R_TURNOPEN);
                    upR = !upR;
                    states.push_back(R_CLOSE);
                }
                states.push_back(F_OPEN);
                states.push_back(B_OPEN);
                states.push_back(L_CW_R_ACW);
                upL = !upL;
                upR = !upR;
                rotated = !rotated;
                states.push_back(F_CLOSE);
                states.push_back(B_CLOSE);
                states.push_back(B_ACW);
                upB = !upB;
            }
        }
    }

    
    ROS_INFO_STREAM("--- PROCESSING STATES ---");
    for(auto state: states){
        bool has_timedout = false;
        if( state == L_CW){
            targetLW += WRISTCW;
            msg.data = targetLW;
            pub_targetLW.publish(msg);
            ROS_INFO_STREAM("L_CW");
            while(abs(degLW - targetLW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if( state == L_ACW){
            targetLW += WRISTACW;
            msg.data = targetLW;
            pub_targetLW.publish(msg);
            ROS_INFO_STREAM("L_ACW");
            while(abs(degLW - targetLW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if( state == R_CW){
            targetRW += WRISTCW;
            msg.data = targetRW;
            pub_targetRW.publish(msg);
            ROS_INFO_STREAM("R_CW");
            while(abs(degRW - targetRW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if( state == R_ACW){
            targetRW += WRISTACW;
            msg.data = targetRW;
            pub_targetRW.publish(msg);
            ROS_INFO_STREAM("R_ACW");
            while(abs(degRW - targetRW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if( state == F_CW){
            targetFW += WRISTCW;
            msg.data = targetFW;
            pub_targetFW.publish(msg);
            ROS_INFO_STREAM("F_CW");
            while(abs(degFW - targetFW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if( state == F_ACW){
            targetFW += WRISTACW;
            msg.data = targetFW;
            pub_targetFW.publish(msg);
            ROS_INFO_STREAM("F_ACW");
            while(abs(degFW - targetFW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if( state == B_CW){
            targetBW += WRISTCW;
            msg.data = targetBW;
            pub_targetBW.publish(msg);
            ROS_INFO_STREAM("B_CW");
            while(abs(degBW - targetBW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if( state == B_ACW){
            targetBW += WRISTACW;
            msg.data = targetBW;
            pub_targetBW.publish(msg);
            ROS_INFO_STREAM("B_ACW");
            while(abs(degBW - targetBW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if(state == L_OPEN){
            targetLH = HANDOPEN;
            msg.data = targetLH;
            pub_targetLH.publish(msg);
            ROS_INFO_STREAM("L_OPEN");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degLH - targetLH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degLH;
                 pub_targetLH.publish(msg);
            }
            
        }else if(state == L_CLOSE){
            targetLH = HANDCLOSE;
            msg.data = targetLH;
            pub_targetLH.publish(msg);
            ROS_INFO_STREAM("L_CLOSE");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degLH - targetLH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degLH;
                 pub_targetLH.publish(msg);
            }

        }else if(state == R_OPEN){
            targetRH = HANDOPEN;
            msg.data = targetRH;
            pub_targetRH.publish(msg);
            ROS_INFO_STREAM("R_OPEN");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degRH - targetRH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degRH;
                 pub_targetRH.publish(msg);
            }

        }else if(state == R_CLOSE){
            targetRH = HANDCLOSE;
            msg.data = targetRH;
            pub_targetRH.publish(msg);
            ROS_INFO_STREAM("R_CLOSE");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degRH - targetRH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degRH;
                 pub_targetRH.publish(msg);
            }

        }else if(state == F_OPEN){
            targetFH = HANDOPEN;
            msg.data = targetFH;
            pub_targetFH.publish(msg);
            ROS_INFO_STREAM("F_OPEN");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degFH - targetFH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degFH;
                 pub_targetFH.publish(msg);
            }

        }else if(state == F_CLOSE){
            targetFH = HANDCLOSE;
            msg.data = targetFH;
            pub_targetFH.publish(msg);
            ROS_INFO_STREAM("F_CLOSE");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degFH - targetFH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degFH;
                 pub_targetFH.publish(msg);
            }

        }else if(state == B_OPEN){
            targetBH = HANDOPEN;
            msg.data = targetBH;
            pub_targetBH.publish(msg);
            ROS_INFO_STREAM("B_OPEN");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degBH - targetBH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degBH;
                 pub_targetBH.publish(msg);
            }

        }else if(state == B_CLOSE){
            targetBH = HANDCLOSE;
            msg.data = targetBH;
            pub_targetBH.publish(msg);
            ROS_INFO_STREAM("B_CLOSE");
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degBH - targetBH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degBH;
                 pub_targetBH.publish(msg);
            }

        }else if(state == L_TURNOPEN){
            targetLW += WRISTCW;
            targetLH = HANDOPEN;
            msg.data = targetLH;
            pub_targetLH.publish(msg);
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degLH - targetLH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degLH;
                 pub_targetLH.publish(msg);
            }

            msg.data = targetLW;
            pub_targetLW.publish(msg);
            ROS_INFO_STREAM("L_TURNOPEN");
            while(abs(degLW - targetLW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if(state == R_TURNOPEN){
            targetRW += WRISTCW;
            targetRH = HANDOPEN;
            msg.data = targetRH;
            pub_targetRH.publish(msg);
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degRH - targetRH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degRH;
                 pub_targetRH.publish(msg);
            }

            msg.data = targetRW;
            pub_targetRW.publish(msg);
            ROS_INFO_STREAM("R_TURNOPEN");
            while(abs(degRW - targetRW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if(state == F_TURNOPEN){
            targetFW += WRISTCW;
            targetFH = HANDOPEN;
            msg.data = targetFH;
            pub_targetFH.publish(msg);
        //    ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degFH - targetFH) > THRESH){
                ros::spinOnce();
                looper.sleep();
          //      if (ros::WallTime::now() >= timeout)
          //      {
          //          has_timedout = true;
          //          break;
          //      }
            }
          /*  if (has_timedout)
            {
                 msg.data = degFH;
                 pub_targetFH.publish(msg);
            }
*/

            msg.data = targetFW;
            pub_targetFW.publish(msg);
            ROS_INFO_STREAM("F_TURNOPEN");
            while(abs(degFW - targetFW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if(state == B_TURNOPEN){
            targetBW += WRISTCW;
            targetBH = HANDOPEN;
            msg.data = targetBH;
            pub_targetBH.publish(msg);
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(abs(degBH - targetBH) > THRESH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            if (has_timedout)
            {
                 msg.data = degBH;
                 pub_targetBH.publish(msg);
            }

            msg.data = targetBW;
            pub_targetBW.publish(msg);
            ROS_INFO_STREAM("B_TURNOPEN");
            while(abs(degBW - targetBW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }else if (state == L_CW_R_ACW){
            targetLW += WRISTCW;
            targetRW += WRISTACW;
            msg.data = targetLW;
            pub_targetLW.publish(msg);
            msg.data = targetRW;
            pub_targetRW.publish(msg);
            ROS_INFO_STREAM("L_CW_R_ACW");
            while(abs(degLW - targetLW) > THRESH || abs(degRW - targetRW) > THRESH){
                ros::spinOnce();
                looper.sleep();

            }
        }else if (state == L_ACW_R_CW){
            targetLW += WRISTACW;
            targetRW += WRISTCW;
            msg.data = targetLW;
            pub_targetLW.publish(msg);
            msg.data = targetRW;
            pub_targetRW.publish(msg);
            ROS_INFO_STREAM("L_ACW_R_CW");
            while(abs(degLW - targetLW) > THRESH || abs(degRW - targetRW) > THRESH){
                ros::spinOnce();
                looper.sleep();
            }
        }
    }

    // ================ STOP ===================
    msg_stop.data = true;
    pub_stop.publish(msg_stop);
    ROS_INFO_STREAM("==== MASTER STOPPED ====");

/*    while(ros::ok())
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

    // ====================== RESET ==========================
    std::vector<int> pins = {14 , 6, 15, 13, 7, 12};
    std::vector<int> pins_pwm = {1, 8};
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

    ROS_INFO_STREAM("==== MASTER RESETTED & EXITING ====");
    return 0;
}

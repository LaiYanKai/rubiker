#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include <wiringPi.h>
#include <softPwm.h>

enum State {
    L_CW, L_ACW, L_CLOSE, L_OPEN,
    R_CW, R_ACW, R_CLOSE, R_OPEN,
    F_CW, F_ACW, F_CLOSE, F_OPEN,
    B_CW, B_ACW, B_CLOSE, B_OPEN,
    L_TURNOPEN, R_TURNOPEN, F_TURNOPEN, B_TURNOPEN,
    L_CW_R_ACW, L_ACW_R_CW,
    P_U, P_D, P_M, F_ACW_B_CW
    };

// These will be overwritten by arguments to main
int WRIST = 540;
int HANDOPEN = 50;
int HANDCLOSE = 500;
float TIMEOUT = 0.75;
int OFFLW = 40;
int OFFRW = 20;
int OFFFW = 20;
int OFFBW = 40;
int PUP = 0;
int PMID = 0;
int PDOWN = -500;

bool compLW = false;
bool compRH = false;
bool compLH = false;
bool compRW = false;
bool compFW = false;
bool compBH = false;
bool compFH = false;
bool compBW = false;
bool compPP = false;

void cbCompPP(const std_msgs::Empty::ConstPtr& msg)
{
   compPP = true;
}

void cbCompLW(const std_msgs::Empty::ConstPtr& msg)
{
   compLW = true;
}
void cbCompRH(const std_msgs::Empty::ConstPtr& msg)
{
   compRH = true;
}
void cbCompLH(const std_msgs::Empty::ConstPtr& msg)
{
   compLH = true;
}
void cbCompRW(const std_msgs::Empty::ConstPtr& msg)
{
   compRW = true;
}
void cbCompFW(const std_msgs::Empty::ConstPtr& msg)
{
   compFW = true;
}
void cbCompBH(const std_msgs::Empty::ConstPtr& msg)
{
   compBH = true;
}
void cbCompFH(const std_msgs::Empty::ConstPtr& msg)
{
   compFH = true;
}
void cbCompBW(const std_msgs::Empty::ConstPtr& msg)
{
   compBW = true;
}

void exec_wrist(std::string verbose, int & target, bool & comp, int WRIST, int OFFSET,
    ros::Publisher & pub_target, std_msgs::Int32 & msg_target, ros::Rate & looper)
{
    ROS_INFO_STREAM(verbose);
    target += WRIST;
    msg_target.data = target + OFFSET;
    pub_target.publish(msg_target);
    while (!comp && ros::ok()) { ros::spinOnce(); looper.sleep(); }; comp = false;

    msg_target.data = target;
    pub_target.publish(msg_target);
    while (!comp && ros::ok()) { ros::spinOnce(); looper.sleep(); }; comp = false;
}
void exec_hand(std::string verbose, int & target, bool & comp, int HAND, float TIMEOUT,
    ros::Publisher & pub_target, std_msgs::Int32 & msg_target,
    ros::Publisher & pub_timeout, std_msgs::Empty & msg_timeout, ros::Rate & looper)
{
    ROS_INFO_STREAM(verbose);
    target = HAND;
    msg_target.data = target;
    pub_target.publish(msg_target);

    ros::Time timeout = ros::Time::now() + ros::Duration(TIMEOUT);
    bool has_timedout = false;
    int i = 0;
    while(!comp && ros::ok()) {
        ros::spinOnce();
        looper.sleep();
        ROS_INFO_STREAM(comp);
        if (ros::Time::now() >= timeout) {
            has_timedout = true;
            i++;
            ROS_INFO_STREAM("TIMEOUTED");
        }
    }
    comp = false;

    if (has_timedout)
        pub_timeout.publish(msg_timeout);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nm;

    // Parse arguments
    WRIST = strtol(argv[2], nullptr, 0);
    HANDOPEN = strtol(argv[3], nullptr, 0);
    HANDCLOSE = strtol(argv[4], nullptr, 0);
    TIMEOUT = strtof(argv[5], nullptr);
    OFFLW = strtol(argv[6], nullptr, 0);
    OFFRW = strtol(argv[7], nullptr, 0);
    OFFFW = strtol(argv[8], nullptr, 0);
    OFFBW = strtol(argv[9], nullptr, 0);
    PUP = strtol(argv[10], nullptr, 0);
    PMID = strtol(argv[11], nullptr, 0);
    PDOWN = strtol(argv[12], nullptr, 0);
    ROS_INFO("Master --> I:%s  WRIST:%d  HAND{OC}:%d,%d  TO:%f  OFF{LRFB}:%d,%d,%d,%d  P{UMD}:%d,%d,%d", argv[1], WRIST, HANDOPEN, HANDCLOSE, TIMEOUT, OFFLW, OFFRW, OFFFW, OFFBW, PUP, PMID, PDOWN);

    wiringPiSetupGpio();

    ros::Subscriber sub_compLW = nm.subscribe("compLW", 1, cbCompLW);
    ros::Subscriber sub_compRH = nm.subscribe("compRH", 1, cbCompRH);
    ros::Subscriber sub_compLH = nm.subscribe("compLH", 1, cbCompLH);
    ros::Subscriber sub_compRW = nm.subscribe("compRW", 1, cbCompRW);
    ros::Subscriber sub_compFW = nm.subscribe("compFW", 1, cbCompFW);
    ros::Subscriber sub_compBH = nm.subscribe("compBH", 1, cbCompBH);
    ros::Subscriber sub_compFH = nm.subscribe("compFH", 1, cbCompFH);
    ros::Subscriber sub_compBW = nm.subscribe("compBW", 1, cbCompBW);
    ros::Subscriber sub_compPP = nm.subscribe("compPP", 1, cbCompPP);

    // wait for motors
    while (ros::ok() && !compLW) {ros::spinOnce();}; compLW = false; ROS_INFO("LW acknowledged");
    while (ros::ok() && !compRH) {ros::spinOnce();}; compRH = false; ROS_INFO("RH acknowledged");
    while (ros::ok() && !compLH) {ros::spinOnce();}; compLH = false; ROS_INFO("LH acknowledged");
    while (ros::ok() && !compRW) {ros::spinOnce();}; compRW = false; ROS_INFO("RW acknowledged");
    while (ros::ok() && !compFW) {ros::spinOnce();}; compFW = false; ROS_INFO("FW acknowledged");
    while (ros::ok() && !compBH) {ros::spinOnce();}; compBH = false; ROS_INFO("BH acknowledged");
    while (ros::ok() && !compFH) {ros::spinOnce();}; compFH = false; ROS_INFO("FH acknowledged");
    while (ros::ok() && !compBW) {ros::spinOnce();}; compBW = false; ROS_INFO("BW acknowledged");
    while (ros::ok() && !compPP) {ros::spinOnce();}; compPP = false; ROS_INFO("PP acknowledged");

    if (!ros::ok())
        return 1;

    ros::Publisher pub_targetLW = nm.advertise<std_msgs::Int32>("targetLW", 1, true);
    ros::Publisher pub_targetRH = nm.advertise<std_msgs::Int32>("targetRH", 1, true);
    ros::Publisher pub_targetLH = nm.advertise<std_msgs::Int32>("targetLH", 1, true);
    ros::Publisher pub_targetRW = nm.advertise<std_msgs::Int32>("targetRW", 1, true);
    ros::Publisher pub_targetFW = nm.advertise<std_msgs::Int32>("targetFW", 1, true);
    ros::Publisher pub_targetBH = nm.advertise<std_msgs::Int32>("targetBH", 1, true);
    ros::Publisher pub_targetFH = nm.advertise<std_msgs::Int32>("targetFH", 1, true);
    ros::Publisher pub_targetBW = nm.advertise<std_msgs::Int32>("targetBW", 1, true);
    ros::Publisher pub_targetPP = nm.advertise<std_msgs::Int32>("targetPP", 1, true);

    ros::Publisher pub_timeoutLW = nm.advertise<std_msgs::Empty>("timeoutLW", 1, true);
    ros::Publisher pub_timeoutRH = nm.advertise<std_msgs::Empty>("timeoutRH", 1, true);
    ros::Publisher pub_timeoutLH = nm.advertise<std_msgs::Empty>("timeoutLH", 1, true);
    ros::Publisher pub_timeoutRW = nm.advertise<std_msgs::Empty>("timeoutRW", 1, true);
    ros::Publisher pub_timeoutFW = nm.advertise<std_msgs::Empty>("timeoutFW", 1, true);
    ros::Publisher pub_timeoutBH = nm.advertise<std_msgs::Empty>("timeoutBH", 1, true);
    ros::Publisher pub_timeoutFH = nm.advertise<std_msgs::Empty>("timeoutFH", 1, true);
    ros::Publisher pub_timeoutBW = nm.advertise<std_msgs::Empty>("timeoutBW", 1, true);
    ros::Publisher pub_timeoutPP = nm.advertise<std_msgs::Empty>("timeoutPP", 1, true);

    ros::Publisher pub_stop = nm.advertise<std_msgs::Bool>("stop", 1, true);

    int targetLH = 0, targetLW = 0, targetRH = 0, targetRW = 0,
        targetBH = 0, targetBW = 0, targetFH = 0, targetFW = 0, targetPP = 0;

    ros::Rate looper(50);
    std_msgs::Int32 msg_target;
    std_msgs::Empty msg_timeout;
    std_msgs::Bool msg_stop;

    bool upL = true, upR = true, upF = true, upB = true,
         cL = true, cR = true, cF = true, cB = true, rotated = false;

    std::string instructions(argv[1]); //{'L','l','R','r','F','f','B','b','T','t','D','d'};
    std::vector<State>states = {};

    for(auto instruction: instructions)
    {
        ROS_INFO_STREAM("Instruction: " << instruction);
	if (instruction == 'P'){
            if(!rotated){
                if(!upF){
                    states.push_back(F_TURNOPEN);
                    states.push_back(F_CLOSE);
                }
		if(!upB){
                    states.push_back(B_TURNOPEN);
                    states.push_back(B_CLOSE);
                }
		if(upL){
                    states.push_back(L_TURNOPEN);
                    states.push_back(L_CLOSE);
                }
		if(upR){
                    states.push_back(R_TURNOPEN);
                    states.push_back(R_CLOSE);
		}
		    states.push_back(F_OPEN);
                    states.push_back(B_OPEN);
                    states.push_back(P_M);
                    states.push_back(L_TURNOPEN);
                    states.push_back(R_TURNOPEN);
                    states.push_back(P_U);
            }else{
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
                if(!upF){
                    states.push_back(F_TURNOPEN);
		    states.push_back(F_CLOSE);
                }
		if(!upB){
                    states.push_back(B_TURNOPEN);
                    states.push_back(B_CLOSE);
                }
		if(upL){
                    states.push_back(L_TURNOPEN);
                    states.push_back(L_CLOSE);
                }
		if(upR){
                    states.push_back(R_TURNOPEN);
                    states.push_back(R_CLOSE);
		}
	        states.push_back(F_OPEN);
                states.push_back(B_OPEN);
                states.push_back(P_M);
                states.push_back(L_TURNOPEN);
                states.push_back(R_TURNOPEN);
                states.push_back(P_U);
            }
	}
        else if(instruction == 'p'){
            states.push_back(L_ACW_R_CW);
            states.push_back(L_CLOSE);
            states.push_back(R_CLOSE);
            states.push_back(P_D);
            states.push_back(F_CLOSE);
            states.push_back(B_CLOSE);
            states.push_back(L_TURNOPEN);
            states.push_back(R_TURNOPEN);
            states.push_back(L_CLOSE);
            states.push_back(R_CLOSE);

        }
	else if(instruction == 'C'){
            states.push_back(L_TURNOPEN);
            states.push_back(L_CLOSE);
            states.push_back(R_TURNOPEN);
            states.push_back(R_CLOSE);
            states.push_back(F_OPEN);
            states.push_back(B_OPEN);
            //take pic
            states.push_back(L_ACW_R_CW);
            states.push_back(F_CLOSE);
            states.push_back(B_CLOSE);
            states.push_back(L_TURNOPEN);
            states.push_back(L_CLOSE);
            states.push_back(R_TURNOPEN);
            states.push_back(R_CLOSE);
            states.push_back(F_OPEN);
            states.push_back(B_OPEN);
            //take pic
            states.push_back(L_ACW_R_CW);
            states.push_back(F_CLOSE);
            states.push_back(B_CLOSE);
            states.push_back(L_TURNOPEN);
            states.push_back(L_CLOSE);
            states.push_back(R_TURNOPEN);
            states.push_back(R_CLOSE);
            states.push_back(F_OPEN);
            states.push_back(B_OPEN);
            //take pic
            states.push_back(L_ACW_R_CW);
            states.push_back(F_CLOSE);
            states.push_back(B_CLOSE);
            states.push_back(L_TURNOPEN);
            states.push_back(L_CLOSE);
            states.push_back(R_TURNOPEN);
            states.push_back(R_CLOSE);
            states.push_back(F_OPEN);
            states.push_back(B_OPEN);
            //take_pic
            states.push_back(F_CLOSE);
            states.push_back(B_CLOSE);
            states.push_back(L_TURNOPEN);
            states.push_back(R_TURNOPEN);
            states.push_back(F_ACW_B_CW);
            //take pic
            states.push_back(F_ACW_B_CW);
            states.push_back(F_ACW_B_CW);
            //take pic
            states.push_back(F_ACW_B_CW);
            states.push_back(L_CLOSE);
            states.push_back(R_CLOSE);

        }
        else if(instruction == 'L'){
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

    ROS_INFO_STREAM("--- EXECUTING STATES ---");
    for(auto state: states){
        if (!ros::ok)
             return 1;

        bool has_timedout = false;
        if( state == L_CW){
            exec_wrist("L_CW", targetLW, compLW, -WRIST, -OFFLW, pub_targetLW, msg_target, looper);
        }else if( state == L_ACW){
            exec_wrist("L_ACW", targetLW, compLW, WRIST, OFFLW, pub_targetLW, msg_target, looper);
        }else if( state == R_CW){
            exec_wrist("R_CW", targetRW, compRW, -WRIST, -OFFRW, pub_targetRW, msg_target, looper);
        }else if( state == R_ACW){
            exec_wrist("R_ACW", targetRW, compRW, WRIST, OFFRW, pub_targetRW, msg_target, looper);
        }else if( state == F_CW){
            exec_wrist("F_CW", targetFW, compFW, -WRIST, -OFFFW, pub_targetFW, msg_target, looper);
        }else if( state == F_ACW){
            exec_wrist("F_ACW", targetFW, compFW, WRIST, OFFFW, pub_targetFW, msg_target, looper);
        }else if( state == B_CW){
            exec_wrist("B_CW", targetBW, compBW, -WRIST, -OFFBW, pub_targetBW, msg_target, looper);
        }else if( state == B_ACW){
            exec_wrist("B_ACW", targetBW, compBW, WRIST, OFFBW, pub_targetBW, msg_target, looper);

        }else if(state == L_OPEN){
            exec_hand("L_OPEN", targetLH, compLH, HANDOPEN, TIMEOUT, pub_targetLH, msg_target, pub_timeoutLH, msg_timeout, looper);
        }else if(state == L_CLOSE){
            exec_hand("L_CLOSE", targetLH, compLH, HANDCLOSE, TIMEOUT, pub_targetLH, msg_target, pub_timeoutLH, msg_timeout, looper);
        }else if(state == R_OPEN){
            exec_hand("R_OPEN", targetRH, compRH, HANDOPEN, TIMEOUT, pub_targetRH, msg_target, pub_timeoutRH, msg_timeout, looper);
        }else if(state == R_CLOSE){
            exec_hand("R_CLOSE", targetRH, compRH, HANDCLOSE, TIMEOUT, pub_targetRH, msg_target, pub_timeoutRH, msg_timeout, looper);
        }else if(state == F_OPEN){
            exec_hand("F_OPEN", targetFH, compFH, HANDOPEN, TIMEOUT, pub_targetFH, msg_target, pub_timeoutFH, msg_timeout, looper);
        }else if(state == F_CLOSE){
            exec_hand("F_CLOSE", targetFH, compFH, HANDCLOSE, TIMEOUT, pub_targetFH, msg_target, pub_timeoutFH, msg_timeout, looper);
        }else if(state == B_OPEN){
            exec_hand("B_OPEN", targetBH, compBH, HANDOPEN, TIMEOUT, pub_targetBH, msg_target, pub_timeoutBH, msg_timeout, looper);
        }else if(state == B_CLOSE){
            exec_hand("B_CLOSE", targetBH, compBH, HANDCLOSE, TIMEOUT, pub_targetBH, msg_target, pub_timeoutBH, msg_timeout, looper);

        }else if(state == L_TURNOPEN){
            targetLH = HANDOPEN;
            msg_target.data = targetLH;
            pub_targetLH.publish(msg_target);
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
int i = 0;
            while(ros::ok() && !compLH){
                ros::spinOnce();
                looper.sleep();
++i;
                if (ros::WallTime::now() >= timeout)
                {
ROS_INFO_STREAM(i);
                    has_timedout = true;
                    break;
                }
            }
            compLH = false;
            if (has_timedout)
            {
                 pub_timeoutLH.publish(msg_timeout);
            }

            targetLW -= WRIST;
            msg_target.data = targetLW - OFFLW; pub_targetLW.publish(msg_target);
            ROS_INFO_STREAM("L_TURNOPEN");
            while(ros::ok() && !compLW) { ros::spinOnce(); looper.sleep(); }; compLW = false;
            msg_target.data = targetLW; pub_targetLW.publish(msg_target);
            while(ros::ok() && !compLW) { ros::spinOnce(); looper.sleep(); }; compLW = false;

        }else if(state == R_TURNOPEN){
            targetRH = HANDOPEN;
            msg_target.data = targetRH;
            pub_targetRH.publish(msg_target);
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(ros::ok() && !compRH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            compRH = false;
            if (has_timedout)
            {
                 pub_timeoutRH.publish(msg_timeout);
            }

            targetRW -= WRIST;
            msg_target.data = targetRW - OFFRW; pub_targetRW.publish(msg_target);
            ROS_INFO_STREAM("R_TURNOPEN");
            while(ros::ok() && !compRW) { ros::spinOnce(); looper.sleep(); }; compRW = false;
            msg_target.data = targetRW; pub_targetRW.publish(msg_target);
            while(ros::ok() && !compRW) { ros::spinOnce(); looper.sleep(); }; compRW = false;
        }
            else if(state == F_TURNOPEN){
            targetFH = HANDOPEN;
            msg_target.data = targetFH;
            pub_targetFH.publish(msg_target);
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(ros::ok() && !compFH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            compFH = false;
            if (has_timedout)
            {
                 pub_timeoutFH.publish(msg_timeout);
            }

            targetFW -= WRIST;
            msg_target.data = targetFW - OFFFW; pub_targetFW.publish(msg_target);
            ROS_INFO_STREAM("F_TURNOPEN");
            while(ros::ok() && !compFW) { ros::spinOnce(); looper.sleep(); }; compFW = false;
            msg_target.data = targetFW; pub_targetFW.publish(msg_target);
            while(ros::ok() && !compFW) { ros::spinOnce(); looper.sleep(); }; compFW = false;
        }else if(state == B_TURNOPEN){
            targetBH = HANDOPEN;
            msg_target.data = targetBH;
            pub_targetBH.publish(msg_target);
            ros::WallTime timeout = ros::WallTime::now() + ros::WallDuration(TIMEOUT);
            while(ros::ok() && !compBH){
                ros::spinOnce();
                looper.sleep();
                if (ros::WallTime::now() >= timeout)
                {
                    has_timedout = true;
                    break;
                }
            }
            compBH = false;
            if (has_timedout)
            {
                 pub_timeoutBH.publish(msg_timeout);
            }

            targetBW -= WRIST;
            msg_target.data = targetBW - OFFBW; pub_targetBW.publish(msg_target);
            ROS_INFO_STREAM("B_TURNOPEN");
            while(ros::ok() && !compBW) { ros::spinOnce(); looper.sleep(); }; compBW = false;
            msg_target.data = targetBW; pub_targetBW.publish(msg_target);
            while(ros::ok() && !compBW) { ros::spinOnce(); looper.sleep(); }; compBW = false;

        }else if (state == L_CW_R_ACW){
            targetLW -= WRIST;
            targetRW += WRIST;
            msg_target.data = targetLW;
            pub_targetLW.publish(msg_target);
            msg_target.data = targetRW;
            pub_targetRW.publish(msg_target);
            ROS_INFO_STREAM("L_CW_R_ACW");
            while(ros::ok() && (!compLW || !compRW)){
                ros::spinOnce();
                looper.sleep();
            }
            compLW = false;
            compRW = false;
        }else if (state == L_ACW_R_CW){
            targetLW += WRIST;
            targetRW -= WRIST;
            msg_target.data = targetLW;
            pub_targetLW.publish(msg_target);
            msg_target.data = targetRW;
            pub_targetRW.publish(msg_target);
            ROS_INFO_STREAM("L_ACW_R_CW");
            while(ros::ok() && (!compLW || !compRW)){
                ros::spinOnce();
                looper.sleep();
            }
            compLW = false;
            compRW = false;
        }else if (state == F_ACW_B_CW){
            targetFW += WRIST;
            targetBW -= WRIST;
            msg_target.data = targetFW;
            pub_targetFW.publish(msg_target);
            msg_target.data = targetBW;
            pub_targetBW.publish(msg_target);
            ROS_INFO_STREAM("F_ACW_B_CW");
            while(ros::ok() && (!compFW || !compBW)){
                ros::spinOnce();
                looper.sleep();
            }
            compFW = false;
            compBW = false;
        }else if (state == P_U){
            targetPP = PUP;
            msg_target.data = targetPP;
            pub_targetPP.publish(msg_target);
            ROS_INFO_STREAM("P_U");
            while(ros::ok() && !compPP){
                ros::spinOnce();
                looper.sleep();
            }
            compPP = false;
        }else if (state == P_M){
            targetPP = PMID;
            msg_target.data = targetPP;
            pub_targetPP.publish(msg_target);
            ROS_INFO_STREAM("P_M");
            while(ros::ok() && !compPP){
                ros::spinOnce();
                looper.sleep();
		ROS_INFO_STREAM(compPP << targetPP);
            }
            compPP = false;
        }else if (state == P_D){
            targetPP = PDOWN;
            msg_target.data = targetPP;
            pub_targetPP.publish(msg_target);
            while(ros::ok() && !compPP){
                ros::spinOnce();
                looper.sleep();
            }
        }
    }

    // ================ STOP ===================
    msg_stop.data = true;
    pub_stop.publish(msg_stop);
    ROS_INFO_STREAM("==== MASTER STOPPED ====");

    ros::Duration(2).sleep(); // makes sure the stop is published
//    ros::spin();
    return 0;

}

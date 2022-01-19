#include <ros/ros.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include <wiringPi.h>
#include <softPwm.h>
#include "rubiker/MotorCmd.h"
#include "rubiker/MotorAck.h"

enum State { L_CW, L_ACW, L_CLOSE, L_OPEN, 
  R_CW,  R_ACW,  R_CLOSE,  R_OPEN,  
  F_CW,  F_ACW,  F_CLOSE,  F_OPEN,
  B_CW,  B_ACW,  B_CLOSE,  B_OPEN,
  L_TURNOPEN,  R_TURNOPEN,  F_TURNOPEN,  B_TURNOPEN,
  L_CW_R_ACW,  L_ACW_R_CW,
  P_U,  P_D,
  F_ACW_B_CW,  F_CW_B_ACW,
};

// These will be overwritten by arguments to main
int CAL = 20;
float CALDUR = 1.0;
float CALSPD = 50;
int WRIST = 540;
int HANDOPEN = 50;
int HANDCLOSE = 500;
float TIMEOUT = 0.75;
int OFFLW = 40;
int OFFRW = 20;
int OFFFW = 20;
int OFFBW = 40;
float PTIME = 0.9;

bool ackLW = false;
bool ackRH = false;
bool ackLH = false;
bool ackRW = false;
bool ackFW = false;
bool ackBH = false;
bool ackFH = false;
bool ackBW = false;
bool ackPP = false;

void cbAckPP(const rubiker::MotorAck::ConstPtr &msg) {  ackPP = true;}
void cbAckLW(const rubiker::MotorAck::ConstPtr &msg) {  ackLW = true;}
void cbAckRH(const rubiker::MotorAck::ConstPtr &msg) {  ackRH = true;}
void cbAckLH(const rubiker::MotorAck::ConstPtr &msg) {  ackLH = true;}
void cbAckRW(const rubiker::MotorAck::ConstPtr &msg) {  ackRW = true;}
void cbAckFW(const rubiker::MotorAck::ConstPtr &msg) {  ackFW = true;}
void cbAckBH(const rubiker::MotorAck::ConstPtr &msg) {  ackBH = true;}
void cbAckFH(const rubiker::MotorAck::ConstPtr &msg) {  ackFH = true;}
void cbAckBW(const rubiker::MotorAck::ConstPtr &msg) {  ackBW = true;}

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
  PTIME = strtof(argv[10], nullptr);
  CAL = strtof(argv[11], nullptr);
  CALSPD = strtof(argv[12], nullptr);
  CALDUR = strtof(argv[13], nullptr);
  ROS_INFO("Master --> I:%s  WRIST:%d  HAND{OC}:%d,%d  TO:%f  OFF{LRFB}:%d,%d,%d,%d  PTIME:%f  CAL{DST}:%d,%f,%f", argv[1], WRIST, HANDOPEN, HANDCLOSE, TIMEOUT, OFFLW, OFFRW, OFFFW, OFFBW, PTIME, CAL, CALSPD, CALDUR);

  wiringPiSetupGpio();

  ros::Subscriber sub_ackLW = nm.subscribe("ackLW", 1, cbAckLW);
  ros::Subscriber sub_ackRH = nm.subscribe("ackRH", 1, cbAckRH);
  ros::Subscriber sub_ackLH = nm.subscribe("ackLH", 1, cbAckLH);
  ros::Subscriber sub_ackRW = nm.subscribe("ackRW", 1, cbAckRW);
  ros::Subscriber sub_ackFW = nm.subscribe("ackFW", 1, cbAckFW);
  ros::Subscriber sub_ackBH = nm.subscribe("ackBH", 1, cbAckBH);
  ros::Subscriber sub_ackFH = nm.subscribe("ackFH", 1, cbAckFH);
  ros::Subscriber sub_ackBW = nm.subscribe("ackBW", 1, cbAckBW);
  ros::Subscriber sub_ackPP = nm.subscribe("ackPP", 1, cbAckPP);

  // wait for motors
  while (ros::ok() && !ackLW) { ros::spinOnce(); };  ackLW = false;  ROS_INFO("LW acknowledged");
  while (ros::ok() && !ackRH) { ros::spinOnce(); };  ackRH = false;  ROS_INFO("RH acknowledged");
  while (ros::ok() && !ackLH) { ros::spinOnce(); };  ackLH = false;  ROS_INFO("LH acknowledged");
  while (ros::ok() && !ackRW) { ros::spinOnce(); };  ackRW = false;  ROS_INFO("RW acknowledged");
  while (ros::ok() && !ackFW) { ros::spinOnce(); };  ackFW = false;  ROS_INFO("FW acknowledged");
  while (ros::ok() && !ackBH) { ros::spinOnce(); };  ackBH = false;  ROS_INFO("BH acknowledged");
  while (ros::ok() && !ackFH) { ros::spinOnce(); };  ackFH = false;  ROS_INFO("FH acknowledged");
  while (ros::ok() && !ackBW) { ros::spinOnce(); };  ackBW = false;  ROS_INFO("BW acknowledged");
  while (ros::ok() && !ackPP) { ros::spinOnce(); };  ackPP = false;  ROS_INFO("PP acknowledged");

  ros::Publisher pub_cmdLW = nm.advertise<std_msgs::Int32>("cmdLW", 1, true);
  ros::Publisher pub_cmdRH = nm.advertise<std_msgs::Int32>("cmdRH", 1, true);
  ros::Publisher pub_cmdLH = nm.advertise<std_msgs::Int32>("cmdLH", 1, true);
  ros::Publisher pub_cmdRW = nm.advertise<std_msgs::Int32>("cmdRW", 1, true);
  ros::Publisher pub_cmdFW = nm.advertise<std_msgs::Int32>("cmdFW", 1, true);
  ros::Publisher pub_cmdBH = nm.advertise<std_msgs::Int32>("cmdBH", 1, true);
  ros::Publisher pub_cmdFH = nm.advertise<std_msgs::Int32>("cmdFH", 1, true);
  ros::Publisher pub_cmdBW = nm.advertise<std_msgs::Int32>("cmdBW", 1, true);
  ros::Publisher pub_cmdPP = nm.advertise<std_msgs::Int32>("cmdPP", 1, true);

  int targetLH = 0, targetLW = 0, targetRH = 0, targetRW = 0,
      targetBH = 0, targetBW = 0, targetFH = 0, targetFW = 0, targetPP = 0;

  ros::Rate looper(20);
  rubiker::MotorCmd msg_cmd;

  bool upL = true, upR = true, upF = true, upB = true,
       cL = true, cR = true, cF = true, cB = true, rotated = false;

  std::string instructions(argv[1]); //{'L','l','R','r','F','f','B','b','T','t','D','d'};
  std::vector<State> states = {};

  auto cmd_timeout = [&](ros::Publisher & pub_cmd, char end) {
    msg_cmd.seq++;
    msg_cmd.type = 'w';
    msg_cmd.end = end;
    msg_cmd.target = 0;
    msg_cmd.speed = 0;
    msg_cmd.duration = 0;
    msg_cmd.relative = false;
    pub_cmd.publish(msg_cmd);
  };
  auto cmd_stop = [&](ros::Publisher & pub_cmd) {
    msg_cmd.seq++;
    msg_cmd.type = 's';
    msg_cmd.end = 'c';
    msg_cmd.target = 0;
    msg_cmd.speed = 0;
    msg_cmd.duration = 0;
    msg_cmd.relative = false;
    pub_cmd.publish(msg_cmd);
  };
  auto cmd_pos = [&](ros::Publisher & pub_cmd, int target, float max_speed, bool relative, char end) {
    msg_cmd.seq++;
    msg_cmd.type = 'p';
    msg_cmd.end = end;
    msg_cmd.target = target;
    msg_cmd.speed = max_speed;
    msg_cmd.relative = relative;
    msg_cmd.duration = 0;
    pub_cmd.publish(msg_cmd);
  };
  auto cmd_time = [&](ros::Publisher & pub_cmd, float duration, float speed, char end) {
    msg_cmd.seq++;
    msg_cmd.type = 't';
    msg_cmd.end = end;
    msg_cmd.target = 0;
    msg_cmd.speed = speed;
    msg_cmd.relative = false;
    msg_cmd.duration = duration;
    pub_cmd.publish(msg_cmd);
  };
  auto cmd_on = [&](ros::Publisher & pub_cmd, float speed) {
    msg_cmd.seq++;
    msg_cmd.type = 'o';
    msg_cmd.end = 'c';
    msg_cmd.target = 0;
    msg_cmd.speed = speed;
    msg_cmd.relative = false;
    msg_cmd.duration = 0;
    pub_cmd.publish(msg_cmd);
  };
  auto cmd_off = [&](ros::Publisher & pub_cmd, char end) {
    msg_cmd.seq++;
    msg_cmd.type = 'f';
    msg_cmd.end = end;
    msg_cmd.target = 0;
    msg_cmd.speed = 0;
    msg_cmd.relative = false;
    msg_cmd.duration = 0;
    pub_cmd.publish(msg_cmd);
  };
  auto cmd_reset = [&](ros::Publisher & pub_cmd, int target, char end) {
    msg_cmd.seq++;
    msg_cmd.type = 'r';
    msg_cmd.end = end;
    msg_cmd.target = target;
    msg_cmd.speed = 0;
    msg_cmd.relative = false;
    msg_cmd.duration = 0;
    pub_cmd.publish(msg_cmd);
  };

  auto wait_ack = [&](bool &ack) {
    while (!ack && ros::ok()) {
      ros::spinOnce();
      looper.sleep();
    };
    ack = false;
  };
  auto wait_ack_two = [&](bool &ack1, bool &ack2) {
    while (ros::ok() && (!ack1 || !ack2)) {
      ros::spinOnce();
      looper.sleep();
    };
    ack1 = false;
    ack2 = false;
  };
  auto wait_ack_timeout = [&](ros::Publisher & pub_cmd, bool &ack, float timeout_duration, char end) {
    ros::Time timeout = ros::Time::now() + ros::Duration(timeout_duration);
    bool has_timedout = false;
    while (!ack && ros::ok()) {
      ros::spinOnce();
      looper.sleep();
      if (ros::Time::now() >= timeout) {
        has_timedout = true;
        break;
      }
    }
    ack = false;
    if (has_timedout)
      cmd_timeout(pub_cmd, end);
  };
  auto exec_wrist_cal = [&](ros::Publisher & pub_cmd, std::string verbose, int &target, int WRIST, int CAL, float CALDUR, float CALSPD, bool &ack) {
    ROS_INFO_STREAM(verbose);
    target += WRIST;
    cmd_pos(pub_cmd, target + CAL, 100, false, 'h');
    wait_ack(ack);

    cmd_time(pub_cmd, CALDUR, CALSPD, 'h');
    wait_ack(ack);
    cmd_reset(pub_cmd, target, 'h');
    wait_ack(ack);
  };
  auto exec_wrist = [&](ros::Publisher & pub_cmd, std::string verbose, int &target, int WRIST, int OFFSET, bool &ack) {
    ROS_INFO_STREAM(verbose);
    target += WRIST;
    cmd_pos(pub_cmd, target + OFFSET, 100, false, 'h');
    wait_ack(ack);

    cmd_pos(pub_cmd, target, 100, false, 'h');
    wait_ack(ack);
  };
  auto exec_wrist_reset = [&](ros::Publisher & pub_cmd, std::string verbose, int &target, bool &ack) {
    ROS_INFO_STREAM(verbose);
    cmd_time(pub_cmd, 0.5, 50, 'h');
    wait_ack(ack);

    cmd_reset(pub_cmd, target, 'h');
    wait_ack(ack);
  };
  auto exec_hand = [&](ros::Publisher & pub_cmd, std::string verbose, int HAND, int TIMEOUT, bool &ack) {
    ROS_INFO_STREAM(verbose);
    cmd_pos(pub_cmd, HAND, 100, false, 'h');
    wait_ack_timeout(pub_cmd, ack, TIMEOUT, 'h');
  };

  for (auto instruction : instructions) {
    ROS_INFO_STREAM("Instruction: " << instruction);
    if (instruction == 'P') {
      if (!rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          states.push_back(F_CLOSE);
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          states.push_back(B_CLOSE);
        }
        if (upL) {
          states.push_back(L_TURNOPEN);
          states.push_back(L_CLOSE);
        }
        if (upR) {
          states.push_back(R_TURNOPEN);
          states.push_back(R_CLOSE);
        }
        states.push_back(F_OPEN);
        states.push_back(B_OPEN);
        states.push_back(L_TURNOPEN);
        states.push_back(R_TURNOPEN);
        states.push_back(P_U);
      } else {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upF) {
          states.push_back(F_TURNOPEN);
          states.push_back(F_CLOSE);
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          states.push_back(B_CLOSE);
        }
        if (upL) {
          states.push_back(L_TURNOPEN);
          states.push_back(L_CLOSE);
        }
        if (upR) {
          states.push_back(R_TURNOPEN);
          states.push_back(R_CLOSE);
        }
        states.push_back(F_OPEN);
        states.push_back(B_OPEN);
        states.push_back(L_TURNOPEN);
        states.push_back(R_TURNOPEN);
        states.push_back(P_U);
      }
    } else if (instruction == 'p') {
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
    } else if (instruction == 'C') {
      states.push_back(L_TURNOPEN);
      states.push_back(L_CLOSE);
      states.push_back(R_TURNOPEN);
      states.push_back(R_CLOSE);
      states.push_back(F_OPEN);
      states.push_back(B_OPEN);
      //take pic
      states.push_back(F_CLOSE);
      states.push_back(B_CLOSE);
      states.push_back(L_TURNOPEN);
      states.push_back(R_TURNOPEN);
      states.push_back(F_ACW_B_CW);
      //take pic
      states.push_back(F_CW_B_ACW);
      states.push_back(L_CLOSE);
      states.push_back(R_CLOSE);
      states.push_back(F_OPEN);
      states.push_back(B_OPEN);
      states.push_back(L_ACW_R_CW);
      //take pic
      states.push_back(F_CLOSE);
      states.push_back(B_CLOSE);
      states.push_back(L_TURNOPEN);
      states.push_back(L_CLOSE);
      states.push_back(R_TURNOPEN);
      states.push_back(R_CLOSE);
      states.push_back(F_OPEN);
      states.push_back(B_OPEN);
      states.push_back(L_ACW_R_CW);
      //take pic
      states.push_back(L_ACW_R_CW);
      states.push_back(L_ACW_R_CW);
      states.push_back(F_CLOSE);
      states.push_back(B_CLOSE);
      states.push_back(L_TURNOPEN);
      states.push_back(L_CLOSE);
      states.push_back(R_TURNOPEN);
      states.push_back(R_CLOSE);
      states.push_back(F_TURNOPEN);
      states.push_back(F_CLOSE);
      states.push_back(B_TURNOPEN);
      states.push_back(B_CLOSE);
      states.push_back(L_OPEN);
      states.push_back(R_OPEN);
      states.push_back(F_CW_B_ACW);
      //take pic
      states.push_back(F_ACW_B_CW);
      states.push_back(L_CLOSE);
      states.push_back(R_CLOSE);
      states.push_back(F_TURNOPEN);
      states.push_back(F_CLOSE);
      states.push_back(B_TURNOPEN);
      states.push_back(B_CLOSE);
      states.push_back(L_TURNOPEN);
      states.push_back(L_CLOSE);
      states.push_back(R_TURNOPEN);
      states.push_back(R_CLOSE);
      states.push_back(L_ACW_R_CW);
      states.push_back(L_ACW_R_CW);
      // take pic
      states.push_back(L_ACW_R_CW);
      states.push_back(L_ACW_R_CW);
      states.push_back(F_CLOSE);
      states.push_back(B_CLOSE);
      states.push_back(L_TURNOPEN);
      states.push_back(L_CLOSE);
      states.push_back(R_TURNOPEN);
      states.push_back(R_CLOSE);
    } else if (instruction == 'L') {
      if (!rotated) {
        if (!upL) {
          states.push_back(L_CW);
          upL = !upL;
          ROS_INFO_STREAM("L_CW");
        } else if (upF && upB) {
          states.push_back(L_CW);
          upL = !upL;
          ROS_INFO_STREAM("L_CW");
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
            ROS_INFO_STREAM("F_TURNOPEN F_CLOSE");
          }
          if (!upB) {
            states.push_back(B_TURNOPEN);
            upB = !upB;
            states.push_back(B_CLOSE);
            ROS_INFO_STREAM("B_TURNOPEN B_CLOSE");
          }
          states.push_back(L_CW);
          upL = !upL;
          ROS_INFO_STREAM("L_CW");
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
          ROS_INFO_STREAM("F_TURNOPEN");
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
          ROS_INFO_STREAM("B_TURNOPEN");
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        ROS_INFO_STREAM("L_ACW_R_CW");
        if (!upL) {
          states.push_back(L_CW);
          upL = !upL;
          ROS_INFO_STREAM("L_CW");
        } else if (upF && upB) {
          states.push_back(L_CW);
          upL = !upL;
          ROS_INFO_STREAM("L_CW");
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
            ROS_INFO_STREAM("F_TURNOPEN F_CLOSE");
          }
          if (!upB) {
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
    } else if (instruction == 'l') {
      if (!rotated) {
        if (!upL) {
          states.push_back(L_ACW);
          upL = !upL;
        } else if (upF && upB) {
          states.push_back(L_ACW);
          upL = !upL;
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
          }
          if (!upB) {
            states.push_back(B_TURNOPEN);
            upB = !upB;
            states.push_back(B_CLOSE);
          }
          states.push_back(L_ACW);
          upL = !upL;
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upL) {
          states.push_back(L_ACW);
          upL = !upL;
        } else if (upF && upB) {
          states.push_back(L_ACW);
          upL = !upL;
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
          }
          if (!upB) {
            states.push_back(B_TURNOPEN);
            upB = !upB;
            states.push_back(B_CLOSE);
          }
          states.push_back(L_ACW);
          upL = !upL;
        }
      }
    } else if (instruction == 'R') {
      if (!rotated) {
        if (!upR) {
          states.push_back(R_CW);
          upR = !upR;
        } else if (upF && upB) {
          states.push_back(R_CW);
          upR = !upR;
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
          }
          if (!upB) {
            states.push_back(B_TURNOPEN);
            upB = !upB;
            states.push_back(B_CLOSE);
          }
          states.push_back(R_CW);
          upR = !upR;
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upR) {
          states.push_back(R_CW);
          upR = !upR;
        } else if (upF && upB) {
          states.push_back(R_CW);
          upR = !upR;
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
          }
          if (!upB) {
            states.push_back(B_TURNOPEN);
            upB = !upB;
            states.push_back(B_CLOSE);
          }
          states.push_back(R_CW);
          upR = !upR;
        }
      }
    } else if (instruction == 'r') {
      if (!rotated) {
        if (!upR) {
          states.push_back(R_ACW);
          upR = !upR;
        } else if (upF && upB) {
          states.push_back(R_ACW);
          upR = !upR;
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
          }
          if (!upB) {
            states.push_back(B_TURNOPEN);
            upB = !upB;
            states.push_back(B_CLOSE);
          }
          states.push_back(R_ACW);
          upR = !upR;
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upR) {
          states.push_back(R_ACW);
          upR = !upR;
        } else if (upF && upB) {
          states.push_back(R_ACW);
          upR = !upR;
        } else {
          if (!upF) {
            states.push_back(F_TURNOPEN);
            upF = !upF;
            states.push_back(F_CLOSE);
          }
          if (!upB) {
            states.push_back(B_TURNOPEN);
            upB = !upB;
            states.push_back(B_CLOSE);
          }
          states.push_back(R_ACW);
          upR = !upR;
        }
      }
    } else if (instruction == 'F') {
      if (!rotated) {
        if (!upF) {
          states.push_back(F_CW);
          upF = !upF;
        } else if (upR && upL) {
          states.push_back(F_CW);
          upF = !upF;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(F_CW);
          upF = !upF;
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upF) {
          states.push_back(F_CW);
          upF = !upF;
        } else if (upR && upL) {
          states.push_back(F_CW);
          upF = !upF;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(F_CW);
          upF = !upF;
        }
      }
    } else if (instruction == 'f') {
      if (!rotated) {
        if (!upF) {
          states.push_back(F_ACW);
          upF = !upF;
        } else if (upR && upL) {
          states.push_back(F_ACW);
          upF = !upF;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(F_ACW);
          upF = !upF;
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upF) {
          states.push_back(F_ACW);
          upF = !upF;
        } else if (upR && upL) {
          states.push_back(F_ACW);
          upF = !upF;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(F_ACW);
          upF = !upF;
        }
      }
    } else if (instruction == 'B') {
      if (!rotated) {
        if (!upB) {
          states.push_back(B_CW);
          upB = !upB;
        } else if (upR && upL) {
          states.push_back(B_CW);
          upB = !upB;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(B_CW);
          upB = !upB;
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upB) {
          states.push_back(B_CW);
          upB = !upB;
        } else if (upR && upL) {
          states.push_back(B_CW);
          upB = !upB;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(B_CW);
          upB = !upB;
        }
      }
    } else if (instruction == 'b') {
      if (!rotated) {
        if (!upB) {
          states.push_back(B_ACW);
          upB = !upB;
        } else if (upR && upL) {
          states.push_back(B_ACW);
          upB = !upB;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(B_ACW);
          upB = !upB;
        }
      } else if (rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
        }
        if (upB) {
          states.push_back(B_OPEN);
        }
        if (upF) {
          states.push_back(F_OPEN);
        }
        states.push_back(L_ACW_R_CW);
        upL = !upL;
        upR = !upR;
        states.push_back(F_CLOSE);
        states.push_back(B_CLOSE);
        rotated = !rotated;
        if (!upB) {
          states.push_back(B_ACW);
          upB = !upB;
        } else if (upR && upL) {
          states.push_back(B_ACW);
          upB = !upB;
        } else {
          if (!upR) {
            states.push_back(R_TURNOPEN);
            upR = !upR;
            states.push_back(R_CLOSE);
          }
          if (!upL) {
            states.push_back(L_TURNOPEN);
            upL = !upL;
            states.push_back(L_CLOSE);
          }
          states.push_back(B_ACW);
          upB = !upB;
        }
      }
    } else if (instruction == 'U') {
      if (rotated) {
        states.push_back(F_CW);
        upF = !upF;
      } else if (!rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
          states.push_back(F_CLOSE);
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
          states.push_back(B_CLOSE);
        }
        if (upL) {
          states.push_back(L_TURNOPEN);
          upL = !upL;
          states.push_back(L_CLOSE);
        }
        if (upR) {
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
    } else if (instruction == 'u') {
      if (rotated) {
        states.push_back(F_ACW);
        upF = !upF;
      } else if (!rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
          states.push_back(F_CLOSE);
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
          states.push_back(B_CLOSE);
        }
        if (upL) {
          states.push_back(L_TURNOPEN);
          upL = !upL;
          states.push_back(L_CLOSE);
        }
        if (upR) {
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
    } else if (instruction == 'D') {
      if (rotated) {
        states.push_back(B_CW);
        upB = !upB;
      } else if (!rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
          states.push_back(F_CLOSE);
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
          states.push_back(B_CLOSE);
        }
        if (upL) {
          states.push_back(L_TURNOPEN);
          upL = !upL;
          states.push_back(L_CLOSE);
        }
        if (upR) {
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
    } else if (instruction == 'd') {
      if (rotated) {
        states.push_back(B_ACW);
        upB = !upB;
      } else if (!rotated) {
        if (!upF) {
          states.push_back(F_TURNOPEN);
          upF = !upF;
          states.push_back(F_CLOSE);
        }
        if (!upB) {
          states.push_back(B_TURNOPEN);
          upB = !upB;
          states.push_back(B_CLOSE);
        }
        if (upL) {
          states.push_back(L_TURNOPEN);
          upL = !upL;
          states.push_back(L_CLOSE);
        }
        if (upR) {
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
  for (auto state : states) {
    if (!ros::ok)
      return 1;

    bool has_timedout = false;
    if (state == L_CW) {
      exec_wrist(pub_cmdLW, "L_CW", targetLW, -WRIST, -OFFLW, ackLW);
    } else if (state == L_ACW) {
      exec_wrist(pub_cmdLW, "L_ACW", targetLW, WRIST, OFFLW, ackLW);
    } else if (state == R_CW) {
      exec_wrist(pub_cmdRW, "R_CW", targetRW, -WRIST, -OFFRW, ackRW);
    } else if (state == R_ACW) {
      exec_wrist(pub_cmdRW, "R_ACW", targetRW, WRIST, OFFRW, ackRW);
    } else if (state == F_CW) {
      exec_wrist(pub_cmdFW, "F_CW", targetFW, -WRIST, -OFFFW, ackFW);
    } else if (state == F_ACW) {
      exec_wrist(pub_cmdFW, "F_ACW", targetFW, WRIST, OFFFW, ackFW);
    } else if (state == B_CW) {
      exec_wrist(pub_cmdBW, "B_CW", targetBW, -WRIST, -OFFBW, ackBW);
    } else if (state == B_ACW) {
      exec_wrist(pub_cmdBW, "B_ACW", targetBW, WRIST, OFFBW, ackBW);
    } else if (state == L_OPEN) {
      exec_hand(pub_cmdLH, "L_OPEN", HANDOPEN, TIMEOUT, ackLH);
      exec_wrist_cal(pub_cmdLW, "L_OW", targetLW, 0, -CAL, CALDUR, CALSPD, ackLW);
    } else if (state == L_CLOSE) {
      exec_hand(pub_cmdLH, "L_CLOSE", HANDCLOSE, TIMEOUT, ackLH);
    } else if (state == R_OPEN) {
      exec_hand(pub_cmdRH, "R_OPEN", HANDOPEN, TIMEOUT, ackRH);
      exec_wrist_cal(pub_cmdRW, "B_OW", targetRW, 0, -CAL, CALDUR, CALSPD, ackRW);
    } else if (state == R_CLOSE) {
      exec_hand(pub_cmdRH, "R_CLOSE", HANDCLOSE, TIMEOUT, ackRH);
    } else if (state == F_OPEN) {
      exec_hand(pub_cmdFH, "F_OPEN", HANDOPEN, TIMEOUT, ackFH);
      exec_wrist_cal(pub_cmdFW, "F_OW", targetFW, 0, -CAL, CALDUR, CALSPD, ackFW);
    } else if (state == F_CLOSE) {
      exec_hand(pub_cmdFH, "F_CLOSE", HANDCLOSE, TIMEOUT, ackFH);
    } else if (state == B_OPEN) {
      exec_hand(pub_cmdBH, "B_OPEN", HANDOPEN, TIMEOUT, ackBH);
      exec_wrist_cal(pub_cmdBW, "B_OW", targetBW, 0, -CAL, CALDUR, CALSPD, ackBW);
    } else if (state == B_CLOSE) {
      exec_hand(pub_cmdBH, "B_CLOSE", HANDCLOSE, TIMEOUT, ackBH);
    } else if (state == L_TURNOPEN) {
      exec_hand(pub_cmdLH, "L_TOH", HANDOPEN, TIMEOUT, ackLH);
      exec_wrist_cal(pub_cmdLW, "L_TOW", targetLW, -WRIST, -CAL, CALDUR, CALSPD, ackLW);
    } else if (state == R_TURNOPEN) {
      exec_hand(pub_cmdRH, "R_TOH", HANDOPEN, TIMEOUT, ackRH);
      exec_wrist_cal(pub_cmdRW, "R_TOW", targetRW, -WRIST, -CAL, CALDUR, CALSPD, ackRW);
    } else if (state == F_TURNOPEN) {
      exec_hand(pub_cmdFH, "F_TOH", HANDOPEN, TIMEOUT, ackFH);
      exec_wrist_cal(pub_cmdFW, "F_TOW", targetFW, -WRIST, -CAL, CALDUR, CALSPD, ackFW);
    } else if (state == B_TURNOPEN) {
      exec_hand(pub_cmdBH, "B_TOH", HANDOPEN, TIMEOUT, ackBH);
      exec_wrist_cal(pub_cmdBW, "B_TOW", targetBW, -WRIST, -CAL, CALDUR, CALSPD, ackBW);
    } else if (state == L_CW_R_ACW) {
      ROS_INFO_STREAM("L_CW_R_ACW");
      targetLW -= WRIST;
      targetRW += WRIST;
      cmd_pos(pub_cmdLW, targetLW, 100, false, 'h');
      cmd_pos(pub_cmdRW, targetRW, 100, false, 'h');
      wait_ack_two(ackLW, ackRW);
    } else if (state == F_CW_B_ACW) {
      ROS_INFO_STREAM("F_CW_B_ACW");
      targetFW -= WRIST;
      targetBW += WRIST;
      cmd_pos(pub_cmdFW, targetFW, 100, false, 'h');
      cmd_pos(pub_cmdBW, targetBW, 100, false, 'h');
      wait_ack_two(ackFW, ackBW);
    } else if (state == L_ACW_R_CW) {
      ROS_INFO_STREAM("L_ACW_R_CW");
      targetLW += WRIST;
      targetRW -= WRIST;
      cmd_pos(pub_cmdLW, targetLW, 100, false, 'h');
      cmd_pos(pub_cmdRW, targetRW, 100, false, 'h');
      wait_ack_two(ackLW, ackRW);
    } else if (state == F_ACW_B_CW) {
      ROS_INFO_STREAM("F_ACW_B_CW");
      targetFW += WRIST;
      targetBW -= WRIST;
      cmd_pos(pub_cmdFW, targetFW, 100, false, 'h');
      cmd_pos(pub_cmdBW, targetBW, 100, false, 'h');
      wait_ack_two(ackFW, ackBW);
    } else if (state == P_U) {
      ROS_INFO_STREAM("P_U");
      cmd_time(pub_cmdPP, PTIME, -50, 'b');
      wait_ack(ackPP);
    } else if (state == P_D) {
      ROS_INFO_STREAM("P_D");
      cmd_time(pub_cmdPP, PTIME, 50, 'b');
      wait_ack(ackPP);
    }
    ROS_INFO_STREAM("---");
  }

  // ================ STOP ===================
  cmd_stop(pub_cmdLW);
  cmd_stop(pub_cmdRW);
  cmd_stop(pub_cmdFW);
  cmd_stop(pub_cmdBW);
  cmd_stop(pub_cmdLH);
  cmd_stop(pub_cmdRH);
  cmd_stop(pub_cmdFH);
  cmd_stop(pub_cmdBH);
  ROS_INFO_STREAM("==== MASTER STOPPED ====");

  ros::Duration(2).sleep(); // makes sure the stop is published
                            //    ros::spin();
  return 0;
}

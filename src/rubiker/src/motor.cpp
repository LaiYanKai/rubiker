#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "rubiker/MotorCmd.h"
#include "rubiker/MotorAck.h"

unsigned int seq = -1;
bool changed_cmd = true;
bool relative = false;
bool run = true;
int target = 0;
float duration = 0.0;
float speed = 0;
float max_speed = 100;
char state = 'o';
char next_end = 'c';
bool changed_deg = true;

// other global inits
int ACCEPTABLE_ERROR = 0;
int LOOP_RATE = 0;
int PIN_Y = 0;
int PIN_B = 0;
int deg = 0;
bool verbose = false;

void handler_y(void)
{
  if (digitalRead(PIN_B)) {
    deg++;
  } else {
    deg--;
  }
  changed_deg = true;
}
void handler_b(void)
{
  if (digitalRead(PIN_Y)) {
    deg--;
  } else {
    deg++;
  }
  changed_deg = true;
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
void cbCmd(const rubiker::MotorCmd::ConstPtr& msg) {
  seq = msg->seq;
  changed_cmd = true;
  state = msg->type;
  if (state == 's') { // stop
    run = false;
  } else if (state == 't') { // timed
    speed = msg->speed;
    duration = msg->duration;
    next_end = msg->end;
  } else if (state == 'p') { // position
    max_speed = msg->speed; // max speed
    if (max_speed > 100)
      max_speed = 100;
    else if (max_speed < -100)
      max_speed = -100;
    relative = msg->relative;
    if (relative)
      target = deg + msg->target;
    else
      target = msg->target;
    next_end = msg->end;
  } else if (state == 'o') { // on at speed
    speed = msg->speed;
  } else if (state == 'f') { // off
    next_end = msg->end;
  } else if (state == 'r') { // reset degrees
    target = msg->target;
    next_end = msg->end;
  }
}
int main (int argc, char **argv)
{
  ros::init(argc, argv, "motor");
  ros::NodeHandle nh;

  // Parse arguments
  std::string MTR(argv[1]);
  int PIN_K = strtol(argv[2], nullptr, 0);
  int PIN_W = strtol(argv[3], nullptr, 0);
  PIN_Y = strtol(argv[4], nullptr, 0);
  PIN_B = strtol(argv[5], nullptr, 0);
  float KP = strtof(argv[6], nullptr);
  float KI = strtof(argv[7], nullptr);
  float KD = strtof(argv[8], nullptr);
  ACCEPTABLE_ERROR = strtol(argv[9], nullptr, 0);
  LOOP_RATE = strtof(argv[11], nullptr);
  verbose = strtol(argv[12], nullptr, 0);
  ROS_INFO("%2s --> K:%02d  W:%02d  Y:%02d  B:%02d  KP:%9.5f  KI:%9.5f  KD:%9.5f  AE:%d  LRATE:%f  Verbose:%d",
    MTR.c_str(), PIN_K, PIN_W, PIN_Y, PIN_B, KP, KI, KD, ACCEPTABLE_ERROR, LOOP_RATE, verbose);

  // Prepare pins with WiringPi
  if (wiringPiSetupGpio() >  0) { return 1; }
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_B, INPUT);
  if (wiringPiISR(PIN_Y, INT_EDGE_FALLING, &handler_y)  < 0) { ROS_INFO("Pin Y ISR Error"); return 1; }
  if (wiringPiISR(PIN_B, INT_EDGE_FALLING, &handler_b)  < 0) { ROS_INFO("Pin B ISR Error"); return 1; }
  softPwmCreate(PIN_K, 0, 100);
  softPwmCreate(PIN_W, 0, 100);

  // subscribers and publishers
  ros::Subscriber sub_cmd = nh.subscribe("cmd" + MTR, 1, cbCmd);
  ros::Publisher pub_ack = nh.advertise<rubiker::MotorAck>("ack" + MTR, 1, true);
  rubiker::MotorAck msg_ack;

  // Prepare PID variables
  float accum = 0;
  float prev_error = 0;
  float error,  prop, integ, deriv;
  char end;
  ros::Rate rate(LOOP_RATE);

  // lambdas
  auto ack = [&]() { 
    msg_ack.seq = seq; 
    msg_ack.deg = deg;
    pub_ack.publish(msg_ack); 
  }; // acknowledge
  auto pwm = [&]() { // pwm
    if (speed >= 0) {
      if (speed > max_speed) {speed = max_speed;}
      softPwmWrite(PIN_K, 0);
      softPwmWrite(PIN_W, speed);
    } else {
      if (speed < -max_speed) {speed = -max_speed;}
      softPwmWrite(PIN_K, -speed);
      softPwmWrite(PIN_W, 0);
    }
  };
  auto pid = [&]() { // one pid iteration
    error = target - deg;
    prop = KP * error;
    accum += error;
    integ = KI * accum;
    deriv = KD * (error - prev_error);
    prev_error = error;
    speed = prop + integ + deriv;
    if (sign(error) != sign(prev_error))
      accum = 0;
  };
  auto pid_reset = [&]() { // reset pid
    accum = 0;
//    prev_error = deg;
  };
  auto verbose_deg = [&]() { // feedback degrees
    if (verbose && changed_deg) {
      ROS_INFO_STREAM("deg" << MTR << ": " << deg);
      changed_deg = false;
    }
  };

  // Acknowledge
  ack();

  // Main Loop
  while (ros::ok() && run) {
    // write end
    end = next_end;
    changed_cmd = false;

    // handle states
    if (state == 'w') { // timeout (watchdog)
      ROS_INFO_STREAM(MTR << ": Timeout Received");
      target = deg; // for holding at end or next loop pid
      pid_reset(); // for holding at end or next loop pid
      ack();
    } else if (state == 't') { // timed
      max_speed = 100;
      pwm();
      if (verbose) { ROS_INFO_STREAM(MTR << ": Timed " << speed << "\% for " << duration << "s") }; // note speed will be limited max_speed after pwms
      ros::Duration{duration}.sleep();
      verbose_deg();
      target = deg; // for holding at end or next loop pid
      pid_reset(); // for holding at end or next loop pid
      ack();
    } else if (state == 'p') { // position
      if (verbose) { ROS_INFO_STREAM(MTR << ": Target " << target << " at max " << max_speed << "\%") };
      pid_reset();
      while (ros::ok() && run && !changed_cmd) {
        verbose_deg();
        pid();
        pwm();
        if (abs(error) <= ACCEPTABLE_ERROR) {
          ack();
          break;
        }
        rate.sleep();
        ros::spinOnce();
      }
    } else if (state == 'o') { // on at speed
      max_speed = 100;
      pwm();
      ack();
      if (verbose) { ROS_INFO_STREAM(MTR << ": On at " << target << "\%") }; // note speed will be limited max_speed after pwm
      while (ros::ok() && run && !changed_cmd) {
        verbose_deg();
        rate.sleep();
        ros::spinOnce();
      }
      target = deg; // for holding at end or next loop pid
      pid_reset(); // for holding at end or next loop pid
    } else if (state == 'f') {
      ack();
      if (verbose) { ROS_INFO_STREAM(MTR << ": Off") };
    } else if (state == 'r') {
      deg = target;
      ack();
    } else {
      ROS_WARN_STREAM("Invalid state " << state << " received");
    }

    if (changed_cmd) { continue; }

    // handle end action
    if (end == 'h') {
      if (verbose) { ROS_INFO_STREAM(MTR << ": Holding") };
      while (ros::ok() && run && !changed_cmd) {
        verbose_deg();
        pid();
        pwm();
        rate.sleep();
        ros::spinOnce();
      }
    } else if (end == 'c') {
      if (verbose) { ROS_INFO_STREAM(MTR << ": Coasting") };
      softPwmWrite(PIN_K, 0);
      softPwmWrite(PIN_W, 0);
      while (ros::ok() && run && !changed_cmd) {
        verbose_deg();
        rate.sleep();
        ros::spinOnce();
      }
    } else {
      if (verbose) { ROS_INFO_STREAM(MTR << ": Braking") };
      softPwmWrite(PIN_K, 100);
      softPwmWrite(PIN_W, 100);
      while (ros::ok() && run && !changed_cmd) {
        verbose_deg();
        rate.sleep();
        ros::spinOnce();
      }
    }
  }

  softPwmWrite(PIN_K, 0);
  softPwmWrite(PIN_W, 0);
  ROS_INFO_STREAM("Stopped " << MTR);
  return 0;
}


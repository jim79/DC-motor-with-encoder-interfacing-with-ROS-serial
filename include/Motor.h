#ifndef MOTOR_H
#define MOTOR_H

#include<Arduino.h>
#include<geometry_msgs/Twist.h>
#include<Config.h>



// 1. Define the functions for moving the bot : forward, backward, turn left, turn right, stopping
void bot_stop(){
  digitalWrite(LMotorBackward, 0);
  digitalWrite(LMotorForward, 0);
  digitalWrite(RMotorBackward, 0);
  digitalWrite(RMotorForward, 0);
}

void bot_forward(){
  digitalWrite(LMotorForward, 1);
  digitalWrite(LMotorBackward, 0);
  digitalWrite(RMotorForward, 1);
  digitalWrite(RMotorBackward, 0);
  nh.spinOnce();
}

void bot_backward(){
  digitalWrite(LMotorForward, 0);
  digitalWrite(LMotorBackward, 1);
  digitalWrite(RMotorForward, 0);
  digitalWrite(RMotorBackward, 1);
  nh.spinOnce();
}

void bot_left(){
  digitalWrite(LMotorBackward, 0);
  digitalWrite(LMotorForward, 0);
  digitalWrite(RMotorBackward, 0);
  digitalWrite(RMotorForward, 1);
  // delay(2);
  nh.spinOnce();
}

void bot_right(){
  digitalWrite(LMotorBackward, 0);
  digitalWrite(LMotorForward, 1);
  digitalWrite(RMotorBackward, 0);
  digitalWrite(RMotorForward, 0);
  // delay(2);
  nh.spinOnce();
}

// 2. Define the cmd velocity call back function
void command_vel_Callback(const geometry_msgs::Twist&cmd_msg){
    if ((int(abs(cmd_msg.linear.x)) != 0))
    {
      analogWrite(LeftMotorPwm, (abs(int(cmd_msg.linear.x)))%256);      // velocity input from cmd_vel
      analogWrite(RightMotorPwm, (abs(int(cmd_msg.linear.x)))%256);     // velocity input from cmd_vel

      // analogWrite(LeftMotorPwm, left_motor_velocity);      // for future use
      // analogWrite(RightMotorPwm, right_motor_velocity);

      nh.loginfo("setspeed");
    } else {
        bot_stop();
        nh.loginfo("stop");
    }
       if (cmd_msg.linear.x > 0){
          bot_forward();
          nh.loginfo("forward");
       } else if (cmd_msg.linear.x < 0){
          bot_backward();
          nh.loginfo("backward");
       } else if (cmd_msg.angular.z > 0){
          bot_left();
          nh.loginfo("left");
       } else if (cmd_msg.angular.z < 0){
          bot_right();
          nh.loginfo("right");
       }    
          
}

#endif 
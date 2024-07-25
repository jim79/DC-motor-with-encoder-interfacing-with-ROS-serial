#ifndef CONFIG_H
#define CONFIG_H


// 1. Declare the Arduino pins connected to motor, PWM pins, Encoder pins
// Left Motor
int LMotorBackward = 4;
int LMotorForward = 5;
int LeftMotorPwm = 9;
int LMotorEncoderA = 2;
int LMotorEncoderB = A5;

// Right Motor
int RMotorBackward = 6;
int RMotorForward = 7;
int RightMotorPwm = 10;
int RMotorEncoderA = 3;
int RMotorEncoderB = A6;

String LMotorDirection;
String RMotorDirection;


// Instantiate a ROS node handler 
ros::NodeHandle nh;

// int left_motor_velocity = 200;
// int right_motor_velocity = 200;




#endif
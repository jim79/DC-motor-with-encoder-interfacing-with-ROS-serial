/*
DC motor with encoder interfacing using ROS serial.
command velocity is given using teleop keyboard
July 25, 2024
*/

/*
Steps to code
1. Include the required header files
2. Declare variables for publishing encoder ticks
3. Instantiate a ROS Publisher to publish the motor encoder ticks
4. Instantiate a ROS subscriber that subscribes to the topic /cmd_vel
5. Within the Arduino setup function : set the pin modes, initialize the baud rate, initialize the node, 
   initialize the subscribers for command velocity and reset encoders, advertise the left and right motor 
   encoder publishers
6. Within the Arduino loop function : assign data values to encoder tick publishers,
process the subcriber callback (nh.spinOnce()) , add a small delay
Other functionalities are implemented in the header files in the include folder
Motor.h 
- Functions for moving the bot : forward, backward, turn left, turn right, stopping
- cmd velocity call back function
Encoder.h
- Interrupt functions for left and right encoders
- Encoder reset function
Config.h
- Pin declarations : Arduino pins connected to motor, PWM pins, Encoder pins
*/

// 1. Include the required header files
#include<ros.h>
#include<Arduino.h>
#include<geometry_msgs/Twist.h>
#include<Config.h>
#include<Motor.h>
#include<Encoder.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Empty.h>


// 2. Declare variables for publishing encoder ticks
std_msgs::Int32 left_encoder;
std_msgs::Int32 right_encoder;

// 3. Instantiate a ROS Publisher to publish the motor encoder ticks
ros::Publisher left_encoder_publisher("left_encoder_ticks", &left_encoder);
ros::Publisher right_encoder_publisher("right_encoder_ticks", &right_encoder);


// 4. Instantiate a ROS subscriber that subscribes to the topic /cmd_vel 
ros::Subscriber<geometry_msgs::Twist> cmd_velocity_sub("cmd_vel",&command_vel_Callback);
ros::Subscriber<std_msgs::Empty> reset_encoders("reset_encoderLR",&reset_encoder_Callback);


/*5. Within the Arduino setup function : set the pin modes, initialize the baud rate, initialize the node, 
   initialize the subscribers for command velocity and reset encoders, advertise the left and right motor 
   encoder publishers )*/ 
void setup(){
  pinMode(LMotorBackward, OUTPUT);
  pinMode(LMotorForward, OUTPUT);
  pinMode(RMotorBackward, OUTPUT);
  pinMode(RMotorForward, OUTPUT);
  pinMode(LeftMotorPwm, OUTPUT);
  pinMode(RightMotorPwm, OUTPUT);

  pinMode(LMotorEncoderA, INPUT);
  pinMode(LMotorEncoderB, INPUT);
  pinMode(RMotorEncoderA, INPUT);
  pinMode(RMotorEncoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(LMotorEncoderA),left_encoder_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RMotorEncoderA), right_encoder_interrupt, RISING);

  bot_stop();
  // Set Baud Rate and Initialise rosserial node
  nh.getHardware() -> setBaud(115200);
  nh.initNode();
  // Setup Subscribers
  nh.subscribe(cmd_velocity_sub);
  nh.subscribe(reset_encoders);
  // Setup Publishers
  nh.advertise(left_encoder_publisher);
  nh.advertise(right_encoder_publisher);
  nh.spinOnce();
}

/* 6. Within the Arduino loop function : assign data values to encoder tick publishers,
process the subcriber callback (nh.spinOnce()) , add a small delay */ 
void loop(){
  left_encoder.data = total_pulses_left;
  right_encoder.data = total_pulses_right;

  left_encoder_publisher.publish(&left_encoder);
  right_encoder_publisher.publish(&right_encoder);
  nh.spinOnce();
  delay(10);
}




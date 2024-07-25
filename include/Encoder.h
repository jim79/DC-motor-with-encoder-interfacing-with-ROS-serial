#ifndef ENCODER_H
#define ENCODER_H

#include<Config.h>
#include<std_msgs/Empty.h>
#include<Arduino.h>

// Variables for counting the encoder pulses
volatile unsigned long total_pulses_left = 0;
volatile unsigned long total_pulses_right = 0;

void left_encoder_interrupt(){
   total_pulses_left++;
}

void right_encoder_interrupt(){
   total_pulses_right++;
}

void reset_encoder_Callback(const std_msgs::Empty &empty){
   total_pulses_right = 0;
   total_pulses_left = 0;
}

#endif
# DC motor with encoder interfacing using ROS serial

##### Project has been built for Arduino Nano

### Equipment required
1. Arduino Uno or Nano  - 1 nos
2. DC motor with encoder  - 2 nos (I have used [SPG30E-200K DC Geared Motor with Encoder](https://robu.in/product/dc-geared-motor-with-encoder-22rpm/))
3. L298 or L293 motor driver - 1 nos

### Steps to code 
1. Include the required header files
2. Declare variables for publishing encoder ticks
3. Instantiate a ROS Publisher to publish the motor encoder ticks
4. Instantiate a ROS subscriber that subscribes to the topic /cmd_vel
5. Within the Arduino setup function : 
    - Set the pin modes
    - Initialize the baud rate
    - Initialize the node
    - Initialize the subscriber for command velocity
    - Initialize the subscriber for resetting encoders
    - Advertise the left and right motor encoder publishers
6. Within the Arduino loop function : 
    - Assign data values to encoder tick publisher
    - Process the subcriber callback (nh.spinOnce())
    - Add a small delay
### Other functionalities are implemented in the header files in the include folder
#### Motor.h 
- Functions for moving the bot : forward, backward, turn left, turn right, stopping
- cmd velocity call back function
#### Encoder.h
- Interrupt functions for left and right encoders
- Encoder reset function
#### Config.h
- Pin declarations : Arduino pins connected to motor, PWM pins, Encoder pins

### Additional Resources
- [DC motor interfacing with ROS serial](https://github.com/jim79/DC-motor-interfacing-with-ROS-serial)
- [Configure VS Code for rosserial_arduino](https://jim79.github.io/rosserial-arduino-vscode/)
- [Video tutorial - Configure VS Code for rosserial_arduino](https://youtu.be/RZAXBMoWJcE)

### Steps to run the code
1. Launch roscore in a terminal 

```
roscore
```

2. In another terminal type in following to launch a rosserial connection with Arduino 

```
rosrun rosserial_arduino serial_node.py _baud:=115200
```

3. In yet another terminal run _teleop_twist_keyboard_ to control the motor. Since we have interfaced only one motor we can verify the following functionalities - bot_forward, bot_backward and bot_stop. The bot_left and bot_right functionalities can be verified by interfacing one more motor. 

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
4. Launch another terminal and echo the motor encoder tick topics
```
rostopic echo \left_encoder_ticks
rostopic echo \right_encoder_ticks

```
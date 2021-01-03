# Part1: Visual Recognization

Target recognition and use transformation to obtain the position and posture of the target object through the recognition result

## Package: darknet_ros

Package for target object recognition and calculation of target position and posture

## Package: usb_cam

Start the feature pack of two cameras

# Part2：Voice Interface

The issuance of the command and the completion result through voice interaction

## Package: audio_common

Package for obtaining commands and feedback command results

### say.py

File for obtaining commands and feedback of command results

### soundplay_node.py

File to play sound

## Package: robot_voice

### iat_publish

Acquire the voice through the microphone and issue the command file after the timing

# Part3: Move

Movement is divided into platform movement and robotic arm movement

## Package: szar_robot_v2

This package is the Interface of the platform

### smart_car_node.launch

Start the chassis communication node

## Package: mecanum_manipulation

This package is the mobile control of the platform

### keyboard_mecanum_manipulation

Control platform movement by keyboard

### mecanum_manipulation

Control platform movement by Visual Object Recognition

## Package: aubo_robot & industrial_core

 ### moveit_planning_execution.launch

This package is the Interface of the robot arm

## Package: aubo_manipulation

This package is the move control of the robot arm

### robot_arm_manipulation_test

Control robot arm movement by keyboard

### robot_arm_manipulation

Control robot arm movement by Visual Object Recognition


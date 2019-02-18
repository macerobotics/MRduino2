/**
  ******************************************************************************
  * @file    mrduino2.h
  * @author  Mace Robotics (www.macerobotics.com)
  * @Licence MIT Licence
  * @version 0.4
  * @date    05/02/2019
  * @brief   lib for MRduino2 robot
  *
 *******************************************************************************/

#ifndef _MRduino2_H
#define _MRduino2_H

// init robot
void initRobot();

// firmare version of STM32 microcontroller
float firmwareVersion();

// move robot
void forward(int speed);
void back(int speed);
void turnLeft(int speed);
void turnRight(int speed);

// proximity sensor
int proxSensor(void);

// leds
void ledRight(bool on_off);
void ledLeft(bool on_off);

// read the switch
int readSwitch();

// read battery tension
float battery();

// motor control
void motorRight(int speed, int direction);
void motorLeft(int speed, int direction);
void motorsDisable(void);// disable H-bridge
void motorsEnable(void);// enable H-bridge
void motorsBrake(void);// hard brake

// move
void forward_mm(int speed, int distance); // pulling function
void forwardmm(int speed, int distance);  // no pulling function
void back_mm(int speed, int distance);    // pulling function
void backmm(int speed, int distance);     // no pulling function
void stop();
//void robotGo(int speed, int coord_X, int coord_Y);

// turn degree
void turnRight_degree(int speed, unsigned int angle); // pulling function
void turnLeft_degree(int speed, unsigned int angle); // pulling function
void turn_degree(int speed, unsigned int angle);// 0 to 360° - pulling function

// position
float robotPositionX();
float robotPositionY();
float robotPositionOrientation();

// encoder
int encoderLeft();
int encoderRight();

// free gpio
void MRpinMode(int pin, int mode);
void MRpinWrite(int pin, int state);

// reset
void resetUc();

#endif

// end of file
/*
 *  Encoder driver function definitions
 *
 *  Author: James Nugen
 *  Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 *  Additional comments and modifications by: DanielFLopez1620
 *
 */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

// ------------------- DEFINITIONS AND CONSTANTS ------------------------------

// This consider the usage of an Arduino Uno, Arduino Mega or any board
// with a similar pin distribution compatible with the one below.
#ifdef ARDUINO_ENC_COUNTER
  // Pin for left motor encoders (can be changed), should be PORTD pins
  #define LEFT_ENC_PIN_A PD2  //pin 2 (Digital)
  #define LEFT_ENC_PIN_B PD3  //pin 3 (Digital)
  
  // Pin for right motor encoders (can be changed), should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4 (Analog)
  #define RIGHT_ENC_PIN_B PC5   //pin A5 (Analog)
#endif

// ----------------------- FUNTION PROTOTYPES ---------------------------------
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#endif // ENCODER_DRIVER_H

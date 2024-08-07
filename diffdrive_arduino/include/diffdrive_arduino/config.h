/**
 * Taken from: diffdrive_arduino | Articulated Robotics.
 * Based on: https://github.com/ros-controls/ros2_control_demos/tree/master/example_2
 * Also considering: https://github.com/buzzology/diffdrive
 * Additional comments and modifications: DanielFLopez1620
 * Description: Header for the  description of the configuration params.
 */

#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

// --------------------- CPP STANDARD HEADERS DEFINITIONS --------------------
#include <string>  // Standard library for strings

// ------------------------ HEADERS DEFINITIONS ------------------------------

/**
 * Structure oriented to configure the serial communication as it will serve to
 * pass the arguments to the serial objects.
*/
struct Config
{
  std::string left_wheel_name = "left_wheel";    // Continious left joint
  std::string right_wheel_name = "right_wheel";  // Continuious right joint
  float loop_rate = 30;                          // Controller rate (Hz)
  std::string device = "/dev/ttyUSB0";           // COM or Port for Serial Com
  int baud_rate = 57600;                         // Baud rate
  int timeout = 1000;                            // Milliseconds before timeout
  int enc_counts_per_rev = 1920;                 // Enc signals per revolution
};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H
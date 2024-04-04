#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

// --------------------- CPP standard headers required ------------------------
#include <string>

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
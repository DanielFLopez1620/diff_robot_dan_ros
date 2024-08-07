/**
 * Taken from: diffdrive_arduino | Articulated Robotics.
 * Based on: https://github.com/ros-controls/ros2_control_demos/tree/master/example_2
 * Also considering: https://github.com/buzzology/diffdrive
 * Additional comments and modifications: DanielFLopez1620
 * Description: Header for the wheel params.
 */

#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

// ----------------------- CPP Standards Headers required ---------------------
#include <string>  // Standard library for strings

// ---------------------------- Class definitions -----------------------------

/**
 * Define wheel class for diffdrive_arduino controller, which is  oriented to 
 * consider the position/velocity of a certain wheel in a differential robot.
 */
class Wheel
{
public:
    std::string name = "";      // Name of the joint
    int enc = 0;                // Encoder counts
    double cmd = 0;             // Command
    double pos = 0;             // Current position
    double vel = 0;             // Current velocity
    double eff = 0;             // Current effor
    double velSetPt = 0;        // Velocity reference 
    double rads_per_count = 0;  // Radians per count

    // Defaulted constructor
    Wheel() = default;

    // User defined constructor to set name and counts per revolution
    Wheel(const std::string &wheel_name, int counts_per_rev);

    // Set up and create a wheel by setting name and counter per revolution
    void setup(const std::string &wheel_name, int counts_per_rev);

    // Stimate position by considering encoder counts received
    double calcEncAngle();
};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H
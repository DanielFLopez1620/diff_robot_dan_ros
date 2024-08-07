
/**
 * Taken from: diffdrive_arduino | Articulated Robotics.
 * Based on: https://github.com/ros-controls/ros2_control_demos/tree/master/example_2
 * Also considering: https://github.com/buzzology/diffdrive
 * Additional comments and modifications: DanielFLopez1620
 * Description: Wheel object defintions
 */

// ------------------------ CPP standards headers required -------------------- 
#include <cmath>

// ------------------------ Controller's dependencies -------------------------
#include "diffdrive_arduino/wheel.h"


// ------------------------- Class implementations ----------------------------

/**
 * User defined constructor that set up a wheel.
 * 
 * @param wheel_name Name of the joint of the wheel
 * @param counts_per_rev Number of slots per revolution in optical encoder
 * 
*/
Wheel::Wheel(const std::string &wheel_name, int counts_per_rev)
{
    setup(wheel_name, counts_per_rev);

} // Wheel:Wheel()

/**
 * Set up wheel parameters.
 * 
 * @param wheel_name Name of the joint of the wheel
 * @param counts_per_rev Number of slots per revolution in optical encoder
 * 
*/
void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
    name = wheel_name;
    rads_per_count = (2*M_PI)/counts_per_rev;
}

/**
 * Calculate the encoder angle by considering the encoder counts received.
 * 
 * @return Position (radians) of the wheel.
*/
double Wheel::calcEncAngle()
{
    return enc * rads_per_count;
}
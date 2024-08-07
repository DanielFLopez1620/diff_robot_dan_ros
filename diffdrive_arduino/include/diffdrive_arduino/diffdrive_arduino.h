/**
 * Taken from: diffdrive_arduino | Articulated Robotics.
 * Based on: https://github.com/ros-controls/ros2_control_demos/tree/master/example_2
 * Also considering: https://github.com/buzzology/diffdrive
 * Additional comments and modifications: DanielFLopez1620
 * Description: Header for the differential controller.
 */

#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

// --------------- CPP standard headers required ------------------------------
#include <cstring>  // C header for strings

// ---------------- ROS2 headers requried -------------------------------------
#include "rclcpp/rclcpp.hpp"            // ROS2 CLient Library for C++
#include "rclcpp_lifecycle/state.hpp"   // Lifecycle management

// ------------ ROS2 Controller for hardware interface required ---------------
#include "hardware_interface/system_interface.hpp" 
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ------------ Other's controller dependencies -------------------------------
#include "config.h"         // Names params 
#include "wheel.h"          // Wheels objects
#include "arduino_comms.h"  // Commands


// -------------- Namespaces of the program -----------------------------------
using hardware_interface::return_type;


// --------------------- Class for hardware interface -------------------------
class DiffDriveArduino : public hardware_interface::SystemInterface
{
public:
    // Basic void constructor
    DiffDriveArduino();

    // For members initialization and set parameters
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) 
        override;

    // Define and export hardware states
    std::vector<hardware_interface::StateInterface> export_state_interfaces() 
        override;

    // Define and export commands for hardware
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() 
        override;

    // For usage when 'power' is enabled.
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) 
        override;

    // For usage when 'power' is disabled.
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) 
        override;

    // Update the states of the hardware (Getter)
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // Commands the hardware based on the states acquired.
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Configuration object needed to set the serial communication
    Config cfg_;

    // Instance object that contains the methods for the serial commands.
    ArduinoComms arduino_;

    // Instance two wheel objects (differential robot)
    Wheel l_wheel_;
    Wheel r_wheel_;

    // Define a logger for displaying info
    rclcpp::Logger logger_;

    // Create a time object from chrono
    std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H
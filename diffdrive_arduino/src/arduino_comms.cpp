// ---------------------- CPP standard headers required -----------------------
#include <sstream>
#include <cstdlib>

// ----------------------- ROS2 Headers Required ------------------------------
#include <rclcpp/rclcpp.hpp>

// ----------------------- Controller's dependencies --------------------------
#include "diffdrive_arduino/arduino_comms.h"


/**
 * Set up the communication with the Arduino
 * 
 * @param serial_device Name of the port (COM/TTY) that connects with Arduino.
 * @param baud_rate Baud rate of transmission fo serial communication
 * @param timeout_ms Time in milliseocnds before timeout.
*/
void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, 
    int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt);
    serial_conn_.open();
}

/**
 * Send serial message with empty content (\\r)
*/
void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

/**
 * Send message to read encoders and update their value.
 * 
 * @param val_1 Pointer to encoder channel 1 for update
 * @param val_2 Pointer to encoder channel 2 for update
*/
void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    // 'e' for reading encoders
    std::string response = sendMsg("e\r");

    // Process returned string in the serial
    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    // Convert string values to integers and update encoder values
    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
}

/**
 * Send message to set PWM of the motors (0 - 255)
 * 
 * @param val_1 Set point to update in motor 1
 * @param val_2 Set point to update in motor 2
*/
void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

/**
 * Send message to update PID constants.
 * 
 * @param k_p Floating-point proportional constant
 * @param k_d Floating-point differential constant
 * @param k_i Floating-point integral constant
 * @param k_o
*/
void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}


/**
 * Method oriented to communicate the controller with the Arduino via Serial,
 * via sendind string messages.
 * 
 * @param msg_to_send Command to send to the Arduino
 * @param print_output Flag of verbosity
 * 
 * @return String of the response received via serial.
*/
std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}
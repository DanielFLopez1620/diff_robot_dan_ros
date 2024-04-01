#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

// ------------------------- CPP Standard Headers required --------------------
#include <cstring>

// ------------------------- Custom headers required --------------------------
#include <serial/serial.h>

// -------------------------- Classs definitions ------------------------------
class ArduinoComms
{
public:
    // Void constructor
    ArduinoComms() {}

    // User defined constructor that relates with a serial connector
    ArduinoComms(const std::string &serial_device, int32_t baud_rate, 
        int32_t timeout_ms): serial_conn_(serial_device, baud_rate, 
        serial::Timeout::simpleTimeout(timeout_ms))
    {  }

    // Function used by the constructor to set the attributes
    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

    // Send empty message via serial
    void sendEmptyMsg();

    // Send readEncoder message via serial and return their value
    void readEncoderValues(int &val_1, int &val_2);

    // Send message for PWM in motor via serial
    void setMotorValues(int val_1, int val_2);

    // Send message via serial to set PID values
    void setPidValues(float k_p, float k_d, float k_i, float k_o);

    // Check serial connection
    bool connected() const 
    { 
        return serial_conn_.isOpen();
    }

    // Method to sending another message (must be consistent with the Arduino
    // implementation)
    std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
    // Serial connection
    serial::Serial serial_conn_; 
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
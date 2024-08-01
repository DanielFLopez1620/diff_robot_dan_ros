/* 
 * Define single-letter commands that will be sent by the PC over the
 * serial link.
 * 
 * Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 * Additional comments by: DanielFLopez1620
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'  // AnalogRead() case for input signals.
#define GET_BAUDRATE   'b'  // Option to set rate for serial transmission
#define PIN_MODE       'c'  // Case to select of pin
#define DIGITAL_READ   'd'  // DigitalRead() case for input signals
#define READ_ENCODERS  'e'  // Case read encoders of a DC motor
#define MOTOR_SPEEDS   'm'  // Option to set speed of a DC motor
#define MOTOR_RAW_PWM  'o'  // Case for a PWM value specification
#define PING           'p'  // Check connection
#define RESET_ENCODERS 'r'  // Option to set to 0 the value of the encoders
#define SERVO_WRITE    's'  // Case to send an angle pose to a servo
#define SERVO_READ     't'  // Case to obtain the current position of a servo
#define UPDATE_PID     'u'  // Oriented to indicate an update on the PIC const
#define DIGITAL_WRITE  'w'  // DigialWrite() case for output signal
#define ANALOG_WRITE   'x'  // AnalogWrite() case for output signal
#define LEFT            0   // Left motor, actuator, encoders...
#define RIGHT           1   // Right motor, actuator, encoders...

#endif



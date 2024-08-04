/**
 * Motor driver definitions.
 * 
 * Original authors: James Nugen, Patrick Goebel
 * 
 * Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 * 
 * Additional comments and modifications by: DanielFLopez1620
 */

// ----------------------- DEFINITIONS ----------------------------------------

// Usage L298 Driver according motors configurations
#ifdef L298_MOTOR_DRIVER
    #define RIGHT_MOTOR_BACKWARD 5
    #define LEFT_MOTOR_BACKWARD  6
    #define RIGHT_MOTOR_FORWARD  9
    #define LEFT_MOTOR_FORWARD   10
    #define RIGHT_MOTOR_ENABLE 12
    #define LEFT_MOTOR_ENABLE 13
#endif

// ------------------- FUNCTION PROTOTYPES ------------------------------------

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

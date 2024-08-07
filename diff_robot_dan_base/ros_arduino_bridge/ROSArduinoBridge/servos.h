/**
 * 
 * Original Author: Nathaniel Gallinger
 * 
 * Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 * 
 * Additional comments and modifications by: DanielFLopez1620
 */

#ifndef SERVOS_H
#define SERVOS_H

// Number of servos connected to the robot
#define N_SERVOS 2

// This delay in milliseconds determines the pause 
// between each one degree step the servo travels.  Increasing 
// this number will make the servo sweep more slowly.  
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.
int stepDelay [N_SERVOS] = { 0, 0 }; // ms

// Pins considered for the servos
byte servoPins [N_SERVOS] = { 3, 4 };

// Initial Position in degrees (vary according your servo installation)
byte servoInitPosition [N_SERVOS] = { 90, 90 };

/**
 * Class oriented to use a servomotor with given positions to set, or to read
 * and obtain info of them.
 */
class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif // SERVOS_H

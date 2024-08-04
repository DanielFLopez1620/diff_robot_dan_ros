/**
 * 
 * Motor driver definitions where you can add a "#elif defined" block to this
 * file to include support for a particular motor driver.  Then add the 
 * appropriate #define near the top of the main ROSArduinoBridge.ino file.
 * 
 * Original authors: James Nugen, Patrick Goebel
 * 
 * Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 * 
 * Additional comments and modifications by: DanielFLopez1620
 */

#ifdef USE_BASE
   
// First option of driver: VNH5019
#ifdef POLOLU_VNH5019
	// Include polulo library for given motor shield
	#include "DualVNH5019MotorShield.h"

	// Create motor driver object
	DualVNH5019MotorShield drive;
	
	/**
	 * Wrapper for the motor driver initialization.
	 */
	void initMotorController() 
	{
		drive.init();
	}

	/**
	 * Wrapper oriented to set the speed for the motors in the driver motor.
	 * 
	 * @param i If it is zero, it is left motor. Otherwise it is right.
	 * @param spd Speed indicator to set by considering PWM.
	 */
	void setMotorSpeed(int i, int spd) 
	{
		if (i == LEFT) 
			drive.setM1Speed(spd);
		else 
			drive.setM2Speed(spd);
	}

	// A convenience function for setting both motor speeds
	/**
	 * Set the speed for both motors.
	 * 
	 * @param leftSpeed Speed for the left motor.
	 * @param rightSpeed Speed for the right motor.
	 */
	void setMotorSpeeds(int leftSpeed, int rightSpeed) 
	{
		setMotorSpeed(LEFT, leftSpeed);
		setMotorSpeed(RIGHT, rightSpeed);
	}

// Second option for driver: MC33926
#elif defined POLOLU_MC33926

	// Include polulo library for the given motor.
	#include "DualMC33926MotorShield.h"

	// Create driver object
	DualMC33926MotorShield drive;
	
	/**
	 * Wrapper of the motor driver initialization
	 */
	void initMotorController() 
	{
		drive.init();
	}

	/**
	 * Wrapper oriented to set the speed for the motors in the driver motor.
	 * 
	 * @param i If it is zero, it is left motor. Otherwise it is right.
	 * @param spd Speed indicator to set by considering PWM.
	 * 
	 */
	void setMotorSpeed(int i, int spd) 
	{
		if (i == LEFT) 
			drive.setM1Speed(spd);
		else 
			drive.setM2Speed(spd);
	}

	/**
	 * Function that encapsulates the option to sset both motors speed at the
	 * same time.
	 */
	void setMotorSpeeds(int leftSpeed, int rightSpeed) 
	{
		setMotorSpeed(LEFT, leftSpeed);
		setMotorSpeed(RIGHT, rightSpeed);
	}

// Third option: Usage of the L298 motor driver
#elif defined L298_MOTOR_DRIVER

	/**
	 * Initialize motors by enabling both.
	 */
	void initMotorController() 
	{
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
		digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
	}
	
	/**
	 * Setter for the speed of the given motor. 
	 * 
	 * @param i If it is zero, it is for left motor. Othersie right motor.
	 * @param spd Considering PWM port, specify the value (0 - 255)
	 */
	void setMotorSpeed(int i, int spd) 
	{
		unsigned char reverse = 0;

		// Set limits (do not overpass 255 or be less that 0)
		if (spd < 0)
		{
			spd = -spd;
			reverse = 1;
		}
		if (spd > 255)
			spd = 255;
		
		// Check which motor should be considered
		if (i == LEFT) 
		{ 
			// Consider direction of movement (given by positive or negative
			// number that should be validated), if it goes forward, it would 
			// set the speed value to the port of forward and turn off the pin
			// for backward. Otherwise, it will do the opposite action.
			if(reverse == 0) 
			{ 
				analogWrite(LEFT_MOTOR_FORWARD, spd); 
				analogWrite(LEFT_MOTOR_BACKWARD, 0); 
			}
			else if (reverse == 1) 
			{ 
				analogWrite(LEFT_MOTOR_BACKWARD, spd); 
				analogWrite(LEFT_MOTOR_FORWARD, 0);
			}
		}
		else
		{
			// Same of the left motor, but applied to the right one.
			if (reverse == 0) 
			{ 
				analogWrite(RIGHT_MOTOR_FORWARD, spd); 
				analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
			else if 
			(reverse == 1) 
			{ 
				analogWrite(RIGHT_MOTOR_BACKWARD, spd); 
				analogWrite(RIGHT_MOTOR_FORWARD, 0); 
			}
		}
	}
	
	/**
	 * Function that groups the set of the velocity for both motors. 
	 * 
	 * @param leftSpeed Value speed for the left motor.
	 * @param rightSpeed Value speed for the right motor.
	 */
	void setMotorSpeeds(int leftSpeed, int rightSpeed) 
	{
		setMotorSpeed(LEFT, leftSpeed);
		setMotorSpeed(RIGHT, rightSpeed);
	}

// If no driver is selected it will return an error.
#else
  	#error A motor driver must be selected!
#endif

#endif // USE_BASE

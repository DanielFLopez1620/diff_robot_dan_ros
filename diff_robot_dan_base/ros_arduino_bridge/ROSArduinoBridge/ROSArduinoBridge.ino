
/**
 * ROSArduinoBridge
 * 
 * A set of simple serial commands to control a differential drive
 * robot and receive back sensor and odometry data. Default 
 * configuration assumes use of an Arduino Mega + Pololu motor
 * controller shield + Robogaia Mega Encoder shield.  Edit the
 * readEncoder() and setMotorSpeed() wrapper functions if using 
 * different motor controller or encoder method.
 *
 * Created for the Pi Robot Project: http://www.pirobot.org
 * and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
 *	
 * Authors: Patrick Goebel, James Nugen
 * 
 * Inspired and modeled after the ArbotiX driver by Michael Ferguson
 * 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2012, Patrick Goebel.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *  copyright notice, this list of conditions and the following
 *  disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 * 
 *  Additional comments and modifications by: DanielFLopez1620
 */

// ------------------------ DEFINITIONS AND MACROS ----------------------------

// Enable the base controller code
#define USE_BASE

// Disable the base controller code
//#undef USE_BASE     

// Define the motor controller and encoder library you are using 

#ifdef USE_BASE
   // The Pololu VNH5019 dual motor driver shield usage
   // #define POLOLU_VNH5019

   // The Pololu MC33926 dual motor driver shield usage
   // #define POLOLU_MC33926

   // The RoboGaia encoder shield usage
   // #define ROBOGAIA
   
   // Encoders are directly attached to Arduino board
   #define ARDUINO_ENC_COUNTER

   // L298 Motor driver usage
   #define L298_MOTOR_DRIVER
#endif

// Enable use of PWM servos as defined in servos.h
// #define USE_SERVOS  

// Disable use of PWM servos
#undef USE_SERVOS     

// Serial port baud rate selected
#define BAUDRATE 57600

// Maximum PWM signal, in the case of the Arduino will be 255
#define MAX_PWM 255

// Arduino library selection for the embedded code
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

// Include definition of serial commands
#include "commands.h"

// Sensor functions
#include "sensors.h"

// Include servo support if required
#ifdef USE_SERVOS
	#include <Servo.h>
	#include "servos.h"
#endif

// If using base, consider the next inclusions
#ifdef USE_BASE
	
	// Motor driver function definitions
	#include "motor_driver.h"

	// Encoder driver function definitions
	#include "encoder_driver.h"

	// PID parameters and functions 
	#include "diff_controller.h"

	// Define PID rate, it will run at 30 times per second (give it in Hz)
	#define PID_RATE 30

	// Convert the rate into an interval
	const int PID_INTERVAL = 1000 / PID_RATE;
	
	// Track the next time we make a PID calculation
	unsigned long nextPID = PID_INTERVAL;

	// Stop the robot if it hasn't received a movement command in some ms
	#define AUTO_STOP_INTERVAL 2000

	
	long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

// --------------- Variable definition and initialization ---------------------

// A pair of varibles to help parse serial commands
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// ----------------------- FUNCTION DEFINITIONS -------------------------------

/**
 * Clear the current commands parameters, as it empties the defined variables
 * for the process of receving/reading the command. 
 */
void resetCommand() 
{
	cmd = NULL;
	memset(argv1, 0, sizeof(argv1));
	memset(argv2, 0, sizeof(argv2));
	arg1 = 0;
	arg2 = 0;
	arg = 0;
	index = 0;
}

/* Run a command.  Commands are defined in commands.h */
/**
 * Interface that considers the value passed via serial to select and process
 * a commands, that are related with the definitions presented in the file:
 * "commands.h". The actions will correspond to the encoder, shield and params
 * selected.
 */
int runCommand() 
{
	int i = 0;
	char *p = argv1;
	char *str;
	int pid_args[4];
	arg1 = atoi(argv1);
	arg2 = atoi(argv2);
	
	// Case structure to select the command to process
	switch(cmd) 
	{
		// Print the baud rate
		case GET_BAUDRATE:
			Serial.println(BAUDRATE);
			break;

		// Read the selected port (as analog)
		case ANALOG_READ:
			Serial.println(analogRead(arg1));
			break;

		// Read the selected port (as digital)
		case DIGITAL_READ:
			Serial.println(digitalRead(arg1));
			break;

		// Write to the given port the value passed (analog case)
		case ANALOG_WRITE:
			analogWrite(arg1, arg2);
			Serial.println("OK"); 
			break;

		// Write to the given port the value passed (digital case)
		case DIGITAL_WRITE:
			if (arg2 == 0) 
				digitalWrite(arg1, LOW);
			else if (arg2 == 1) 
				digitalWrite(arg1, HIGH);
			Serial.println("OK"); 
			break;
		
		// Set pin mode for the pin at the location passed, if it is 0 it will
		// be considered as input, or in case of 1 it will be output.
		case PIN_MODE:
			if (arg2 == 0) 
				pinMode(arg1, INPUT);
			else if (arg2 == 1) 
				pinMode(arg1, OUTPUT);
			Serial.println("OK");
			break;
		
		// Make ping (validate info) of the given port
		case PING:
			Serial.println(Ping(arg1));
			break;
// In case there are servos.
#ifdef USE_SERVOS

		// Write servo position to the given servo port
		case SERVO_WRITE:
			servos[arg1].setTargetPosition(arg2);
			Serial.println("OK");
			break;

		// Read servo position of the given servo port
		case SERVO_READ:
			Serial.println(servos[arg1].getServo().read());
			break;

#endif // USE_SERVOS
    
#ifdef USE_BASE

		// Will return encoders values as two numbers separated by a space, the
		// first value will be the lecture for the left encoder and the second
		// one will be the lecture for the right encoder
		case READ_ENCODERS:
			Serial.print(readEncoder(LEFT));
			Serial.print(" ");
			Serial.println(readEncoder(RIGHT));
			break;

		// Will call the wrappers to reset the PID and encoders
		case RESET_ENCODERS:
			resetEncoders();
			resetPID();
			Serial.println("OK");
			break;
		
		// Add setpoint by considering a target frame to the motor, considered
		// in the form <command> <target_left> <target_right>
		case MOTOR_SPEEDS:
			// Reset autostop timer
			lastMotorCommand = millis();
			if (arg1 == 0 && arg2 == 0) 
			{
				setMotorSpeeds(0, 0);
				resetPID();
				moving = 0;
			}
			else 
				moving = 1;
			leftPID.TargetTicksPerFrame = arg1;
			rightPID.TargetTicksPerFrame = arg2;
			Serial.println("OK"); 
			break;

		// Provide a PWM (value from 0 - 255 or PWM limit) to set the motors
		case MOTOR_RAW_PWM:
			// Reset the auto stop timer
			lastMotorCommand = millis();
			
			// Sneaky way to temporarily disable the PID
			resetPID();
			moving = 0; 
			
			setMotorSpeeds(arg1, arg2);
			Serial.println("OK"); 
			break;

		// Update PID vlaues by providing firs Kp, then Kd, then Ki and finally
		// Ko, should be passed as a string separated by spaces.
		case UPDATE_PID:
			// Loop to read all the values passed
			while ((str = strtok_r(p, ":", &p)) != '\0') 
			{
				pid_args[i] = atoi(str);
				i++;
			}

			// Update values
			Kp = pid_args[0];
			Kd = pid_args[1];
			Ki = pid_args[2];
			Ko = pid_args[3];
			Serial.println("OK");
			break;

#endif // USE_BASE

		default:
			Serial.println("Invalid Command");
			break;
	}
}

// ------------------------- SETUP FUNCTION -----------------------------------

void setup() 
{
  	Serial.begin(BAUDRATE);

// Initialize the motor controller if used
#ifdef USE_BASE
	#ifdef ARDUINO_ENC_COUNTER
		
		// Set as inputs by using Data Direction Registers, in this case for 
		// the port D and port C by using a left shift operation to specify 
		// input, by then using the negation of it and applying an and bitwise.
		DDRD &= ~(1<<LEFT_ENC_PIN_A);
		DDRD &= ~(1<<LEFT_ENC_PIN_B);
		DDRC &= ~(1<<RIGHT_ENC_PIN_A);
		DDRC &= ~(1<<RIGHT_ENC_PIN_B);
		
		// Enable pull up resistors for the encoders pins.
		PORTD |= (1<<LEFT_ENC_PIN_A);
		PORTD |= (1<<LEFT_ENC_PIN_B);
		PORTC |= (1<<RIGHT_ENC_PIN_A);
		PORTC |= (1<<RIGHT_ENC_PIN_B);

		// Disable use of PWM servos by using Pin Change Mask Register as it 
		// enables or disables pin change interrupts. Here it tell pin change
		// mask to listen to left and right encoder pins.
		PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
		PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
		
		// Enable PCINT1 and PCINT2 interrupt in the general interrupt mask 
		// for port C and port D.
		PCICR |= (1 << PCIE1) | (1 << PCIE2);

	#endif // ARDUINO_ENC_COUNTER

	// Initialize controller and reset PID to prevent bad start
	initMotorController();
	resetPID();

#endif // USE_BASE

// Attach servos if they are used
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) 
	{
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

// ---------------------------- LOOOP FUNCTION --------------------------------
/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() 
{
	// Prepare for serial transmission 
	while (Serial.available() > 0) 
	{
		// Read the next character
		chr = Serial.read();

		// Terminate a command with a CR
		if (chr == 13) 
		{
			if (arg == 1) 
				argv1[index] = NULL;
			else if (arg == 2) 
				argv2[index] = NULL;
			runCommand();
			resetCommand();
		}
		// Use spaces to delimit parts of the command
		else if (chr == ' ') 
		{
			// Step through the arguments
			if (arg == 0) 
				arg = 1;
			else if (arg == 1)  
			{
				argv1[index] = NULL;
				arg = 2;
				index = 0;
			}
			continue;
		}
		// Identify if we are in the command, the first or the second arg,
		// then update
		else 
		{
			if (arg == 0) 
			{
				// The first arg is the single-letter command
				cmd = chr;
			}
			else if (arg == 1) 
			{
				// Subsequent arguments can be more than one character
				argv1[index] = chr;
				index++;
			}
			else if (arg == 2) 
			{
				// Add until serial read ends
				argv2[index] = chr;
				index++;
			}
		}
	}
  
// If we are using base control, run a PID calculation at the appropriate 
// intervals
#ifdef USE_BASE
	// Intervene in the appropiate interval
	if (millis() > nextPID) 
	{
		updatePID();
		nextPID += PID_INTERVAL;
	}
	
	// Check to see if we have exceeded the auto-stop interval
	if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
		setMotorSpeeds(0, 0);
		moving = 0;
	}
#endif // USE_BASE

// Sweep servos
#ifdef USE_SERVOS
	int i;
	for (i = 0; i < N_SERVOS; i++) {
		servos[i].doSweep();
	}
#endif // USE_SERVOS
}


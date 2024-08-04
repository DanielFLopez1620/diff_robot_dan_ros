/**
 * Servo Sweep
 * Sweep servos one degree step at a time with a user defined
 * delay in between steps.  Supports changing direction 
 * mid-sweep.  Important for applications such as robotic arms
 * where the stock servo speed is too fast for the strength
 * of your system.
 * 
 * Original Author: Nathaniel Gallinger
 */

#ifdef USE_SERVOS

/**
 * User defined constructor that initializes in zeor the current and target
 * position, and also de sweep command.
 */
SweepServo::SweepServo()
{
	this->currentPositionDegrees = 0;
	this->targetPositionDegrees = 0;
	this->lastSweepCommand = 0;
}


/**
 * Initialization function that attach the servo, updates the initial pose
 * and set the current pose to the intial pose.
 * 
 * @param servoPin Pin connected to the servo
 * @param stepDelayMs Delay to consider in milliseconds
 * @param initPosition Default start position
 */
void SweepServo::initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition)
{
	this->servo.attach(servoPin);
	this->stepDelayMs = stepDelayMs;
	this->currentPositionDegrees = initPosition;
	this->targetPositionDegrees = initPosition;
	this->lastSweepCommand = millis();
}

/**
 * Perform Sweep
 */
void SweepServo::doSweep()
{

	// Get ellapsed time
	int delta = millis() - this->lastSweepCommand;

	// Check if time for a step
	if (delta > this->stepDelayMs) 
	{
		// Check step direction
		if (this->targetPositionDegrees > this->currentPositionDegrees) 
		{
			this->currentPositionDegrees++;
			this->servo.write(this->currentPositionDegrees);
		}
		else if (this->targetPositionDegrees < this->currentPositionDegrees) 
		{
			this->currentPositionDegrees--;
			this->servo.write(this->currentPositionDegrees);
		}
		// If target == current position, do nothing

		// Reset timer
		this->lastSweepCommand = millis();
	}
}


/**
 * Set new target posiiton
 * 
 * @param position Objective pose to move
 */
void SweepServo::setTargetPosition(int position)
{
  	this->targetPositionDegrees = position;
}


/**
 * Wrapper to acces the servo's address pointed
 * 
 * @return Pointer to the object of the class
 */
Servo SweepServo::getServo()
{
  	return this->servo;
}

#endif // USE_SERVOS
/* Functions and type-defs for PID control.
 *
 * 	Original Author: James Nugen, Patrick Goebel
 * 
 *  Taken mostly from Mike Ferguson's ArbotiX code which lives at:
 *  https://github.com/vanadiumlabs/arbotix_ros/blob/noetic-devel/arbotix_firmware/src/diff_controller.h
 *	
 *	First Edit: Taken from Josh Newans Repository:
 *  https://github.com/joshnewans/ros_arduino_bridge/tree/main
 *
 * 
 *  Additional edits and comments: DanielFLopez1620
 */


/**
 * Structure oriented to consider the PID params in terms of position/velocity
 * guided by the encoder counts in a discrete time. It considers:
 * 
 * - Previous Input (PrevInput) instaed of PrevError to avoid derivate kick.
 * 	 http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
 * 
 * - Integrated terms (Iterms) instaed of integrated error (Ierror) for better
 *   tunning changes. 
 *   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes
 */
typedef struct 
{
	double TargetTicksPerFrame;    // Target speed in ticks per frame
	long Encoder;                  // Encoder count
	long PrevEnc;                  // Last encoder count

	int PrevInput;                 // Last input
	//int PrevErr;                 // Last error

	//int Ierror;                  // Integrated error
	int ITerm;                     //integrated term

	long output;                   // Last motor setting
	
} SetPointInfo;

// We need to cover left and right mtoor 
SetPointInfo leftPID, rightPID;

/* PID defualt params considered */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving. In particular, assign both Encoder and
 * PrevEnc the current encoder value.
 * 
 * Consider:
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 */
void resetPID()
{
	// Left reset or reinitialization
	leftPID.TargetTicksPerFrame = 0.0;
	leftPID.Encoder = readEncoder(LEFT);
	leftPID.PrevEnc = leftPID.Encoder;
	leftPID.output = 0;
	leftPID.PrevInput = 0;
	leftPID.ITerm = 0;

	// Right reset or reinitialization
	rightPID.TargetTicksPerFrame = 0.0;
	rightPID.Encoder = readEncoder(RIGHT);
	rightPID.PrevEnc = rightPID.Encoder;
	rightPID.output = 0;
	rightPID.PrevInput = 0;
	rightPID.ITerm = 0;

} // resetPID()

/**
 * PID do routine that considers derivate kick and turning changes:
 * 
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
 * 
 * @param p Setpoint for the PID (related with motor and the side it's located)
 */
void doPID(SetPointInfo * p) 
{
	// Variables considerid for proportional error, input and output 
	long Perror;
	long output;
	int input;

	// Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
	input = p->Encoder - p->PrevEnc;
	Perror = p->TargetTicksPerFrame - input;

	// Avoid derivative kick and allow tuning changes,
	// output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
	// p->PrevErr = Perror;
	output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
	p->PrevEnc = p->Encoder;

	output += p->output;
	
	// Avoid saturation of system (considering both limits)
	if (output >= MAX_PWM)
		output = MAX_PWM;
	else if (output <= -MAX_PWM)
		output = -MAX_PWM;
	else

	// Allow tunning changes
	p->ITerm += Ki * Perror;
	p->output = output;
	p->PrevInput = input;
} // doPID()

/**
 * Read encoders values and call the PID routine.
 * It considers a reset to prevent startup spike.
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * 
 */
/* Read the encoder values and call the PID routine */
void updatePID() 
{
	// Read the encoders
	leftPID.Encoder = readEncoder(LEFT);
	rightPID.Encoder = readEncoder(RIGHT);
	
	// If we aren't moving, reset PID as there is nothing more to do.
	// This is obtained from the PrevInput attribute of the PIDs
	if (!moving)
	{
		if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
		return;
  	}

	// Compute PID update for each motor
	doPID(&rightPID);
	doPID(&leftPID);

	// Set a propoer motor speed
	setMotorSpeeds(leftPID.output, rightPID.output);

} // updatePID()


/**
 * Encoder definitions
 * 
 * Add an "#ifdef" block to this file to include support for a particular
 * encoder board or library. Then add the appropriate #define near the top of
 * the main ROSArduinoBridge.ino file.
 * 
 * Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 * 
 * Additional comments and modifications by: DanielFLopez1620
 * 
 */

#ifdef USE_BASE

#ifdef ROBOGAIA
  
	// The Robogaia Mega Encoder shield
	#include "MegaEncoderCounter.h"

	// Create the encoder shield object
	MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
	
	/**
	 * Wrapper for the encoder reading function
	 * 
	 * @param i If it is 1, it refers to left; otherwise right.
	 * 
	 * @return Count of the selected encoder.
	 */
	long readEncoder(int i) 
	{
		if (i == LEFT) 
			return encoders.YAxisGetCount();
		else 
			return encoders.XAxisGetCount();
	}

	/**
	 * Wrapper for the resent encoder function 
	 * 
	 * @param i If it is 1, it refers to left; otherwise right.
	 * 
	 * @return Check of the reset of the boolean value.
	 */
	void resetEncoder(int i) 
	{
		if (i == LEFT) 
			return encoders.YAxisReset();
		else 
			return encoders.XAxisReset();
	}

// Using definition from the file encoder_driver.ino
#elif defined(ARDUINO_ENC_COUNTER)

	// Define long integer values that facilitate the volatile change
	volatile long left_enc_pos = 0L;
	volatile long right_enc_pos = 0L;

	// Encoder lookup table
	static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  

	/**
	 * Interrupt routine for left encoder, which make the actual counts.
	 */
	ISR (PCINT2_vect)
	{
		// Temporal for reading last encoder lecture
		static uint8_t enc_last = 0;
		
		// Shift previous state two places
		enc_last <<= 2; 

		// Read the current state in the two lowest bits
		enc_last |= (PIND & (3 << 2)) >> 2; 
	
		left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
	}
	
	/* Interrupt routine for RIGHT encoder, taking care of actual counting */
	ISR (PCINT1_vect){
			static uint8_t enc_last=0;
				
		enc_last <<=2; //shift previous state two places
		enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
	
		right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
	}
	
	/* Wrap the encoder reading function */
	long readEncoder(int i) {
		if (i == LEFT) return left_enc_pos;
		else return right_enc_pos;
	}

	/* Wrap the encoder reset function */
	void resetEncoder(int i) 
	{
		if (i == LEFT)
		{
			left_enc_pos=0L;
			return;
		} 
		else 
		{ 
			right_enc_pos=0L;
			return;
		}
	}
#else
  	#error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() 
{
  	resetEncoder(LEFT);
  	resetEncoder(RIGHT);
}

#endif


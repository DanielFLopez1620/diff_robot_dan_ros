/**
 * Encoder definitions
 * 
 * Add an "#ifdef" block to this file to include support for a particular
 * encoder board or library. Then add the appropriate #define near the top of
 * the main ROSArduinoBridge.ino file.
 * 
 * Original Authors: James Nugen, Patrick Goebel
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

	// Create the encoder shield object that initializes the Mega Encoder
	// counter in the 4X Count mode
	MegaEncoderCounter encoders = MegaEncoderCounter(4);
	
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

	// Encoder lookup table to check the direction of the given motor. The
	// basic for this is to consider the quadrature encoder as there are four
	// possible states (00, 01, 10, 11), to detect the direction we need to
	// consider the previous and current state of the encoder, then, if we
	// consider the permutations made during the lectures (two bits from
	// previous and two bits for current state) and we consider the binary 
	// to integer conversion, we can determinate the direction, for example,
	// if I had 01 as previous state and now I have 11, we have the binary
	// '0111' which is seven, and looking for the index in the lookup table we
	// obtain forward movement (1).
	static const int8_t ENC_STATES [] = {
		0,1,-1,0,
		-1,0,0,1,
		1,0,0,-1,
		0,-1,1,0
	};  

	/**
	 * Interrupt routine for left encoder, which make the actual counts by
	 * considering pin change interrupt from vector 1 (pins 0 - 7).
	 */
	ISR (PCINT2_vect)
	{
		// Temporal for reading last encoder lecture
		static uint8_t enc_last = 0;
		
		// Shift previous state two places (make room to new lecture)
		enc_last <<= 2; 

		// Read the current state into lowest 2 bits by first creating a mask
		// (0b00001100) to isolate the bits at pos 2 and 3 of port D, then 
		// applies that bitmask to the portd for extracting the current state
		// of the encoder, then it is shift to the lowest to bits and finally 
		// make the combination with the OR bitwise.
		enc_last |= (PIND & (3 << 2)) >> 2; 
	
		// Extract the lowest 4 bits (previous and current state), after that
		// we use the lookup table to obtain the proper state of the sequence
		// to determinate if it iift previous state two placess going in one 
		// or another direction or if it is stopped.
		left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
	}
	
	/**
	 * Interrupt routine for right encoder, which make the actual counts by
	 * considering pin change interrupt from vector 1 (pins A0 - A5)
	 */
	ISR (PCINT1_vect)
	{
		// Temporal for reading last encoder lecture
		static uint8_t enc_last = 0;
		
		// Shift previous state two places (make room to new lecture)
		enc_last <<= 2; 

		// Read the current state into lowest 2 bits by first creating a mask
		// (0b00110000) to isolate the bits at pos 4 and 5 of port C, then 
		// applies that bitmask to the portc for extracting the current state
		// of the encoder, then it is shift to the lowest to bits and finally 
		// make the combination with the OR bitwise.
		enc_last |= (PINC & (3 << 4)) >> 4;
	
		// Extract the lowest 4 bits (previous and current state), after that
		// we use the lookup table to obtain the proper state of the sequence
		// to determinate if it iift previous state two placess going in one 
		// or another direction or if it is stopped. 
		right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
	}
	
	/**
	 * Wrapper for reading encoder pose of the specified motor.
	 * 
	 * @param i If it is 0, it is left motor. Otherwise right motor.
	 * 
	 * @return Encoder lectures of the given sight
	 */
	long readEncoder(int i) 
	{
		if (i == LEFT) 
			return left_enc_pos;
		else 
			return right_enc_pos;
	}

	/**
	 * Wrap the encoder reset function of the specified motor, as it resets the
	 *  encoder count to zero (but still considering long case type).
	 * 
	 * @param i If it is 0, it is left motor. Otherwise right motor.
	 * 
	 */
	void resetEncoder(int i) 
	{
		if (i == LEFT)
		{
			left_enc_pos=0L;
		} 
		else 
		{ 
			right_enc_pos=0L;
		}
	}

// If no type of encoder is sent, it will launch an error.
#else
  	#error A encoder driver must be selected!
#endif

/**
 * Encapsulates the reset of both encoders counts for the right and left
 * motors.
 */
void resetEncoders() 
{
  	resetEncoder(LEFT);
  	resetEncoder(RIGHT);
}

#endif // USE_BASE
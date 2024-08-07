/* 
 * Functions for various sensor types 
 *
 * Original authors: James Nugen, Patrick Goebel
 * 
 * Taken from: https://github.com/joshnewans/ros_arduino_bridge/tree/main
 * 
 * Additional comments and modifications by: DanielFLopez1620
 */

/**
 * Function oriented to read a ultrasound sensor which considers that the speed
 * of sound is 340 m/s or 29 microseconds per cm. The ping travels out and 
 * back, so to find the distance of the object we take half of the distance 
 * travelled.
 * 
 * @param microseconds Ms that last the ultrasound signal to return.
 * 
 * @return Measured distances in cm.
 */
float microsecondsToCm(long microseconds)
{
	return microseconds / 29 / 2;
}

/**
 * Custom imlementation of a ping, which is intended to test for a few
 * microseconds a seonsro, for example, a ultrasonic one, to achieve the
 * distance read by the time of return of the pulse sent.
 * 
 * @param pin Pin connected to the sensor.
 */
long Ping(int pin) 
{
	long duration, range;

	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds(2);
	digitalWrite(pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(pin, LOW);

	pinMode(pin, INPUT);
	duration = pulseIn(pin, HIGH);

	range = microsecondsToCm(duration);
	
	return(range);
}


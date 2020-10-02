/*
 * pid.h
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// Define PID (PI) state variables
typedef struct PID_t {
	float kp; // Proportional tuning constant
	float ki; // Integral tuning constant

	float error; // last error value
	float I;     // running integral of error
	float dt;    // time interval for updates

	const char * tag; // tag label for debug messages


} PID;


// public module API functions
float pidUpdate(float target, float current, PID * pid_state); // update state of PID and return new output


extern PID pid_left;
extern PID pid_right;

#endif /* INC_PID_H_ */

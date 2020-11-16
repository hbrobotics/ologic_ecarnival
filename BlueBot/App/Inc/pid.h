/*
 * pid.h
 *
 *  Implement PI controller
 *
 *  Created on: Sep 16, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdbool.h>

// Define PID (PI) state variables
typedef	struct PID_STATE_t {
	float error; // last error value
	float I;     // running integral of error

	float ref;  // last setpoint
	float fb;   // last feed back
	float u;    // last output
} PID_STATE;


// Define PID Configuration including state
typedef struct PID_t {
	float kp; // Proportional tuning constant
	float ki; // Integral tuning constant

	float dt;    // time interval for updates

	bool openLoop; // set true PID update is open loop pass through mode
	const char * tag; // tag label for debug messages


	PID_STATE state; // current state of the controller

} PID;


// public module API functions
float pidUpdate(float target, float current, PID * pid); // update state of PID and return new output

// Controller can be set to open loop to gather open loop data to use for tuning
inline bool setOpenLoop(PID * pid, bool openLoop) { pid->openLoop=openLoop; return openLoop; };

// refeneces to PID state variables
extern PID pid_left;
extern PID pid_right;

#endif /* INC_PID_H_ */

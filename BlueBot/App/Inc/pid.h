/*
 * pid.h
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct PID_t {
	float kp;
	float ki;
	float error;
	float I;

	const char * tag;
	float dt;

} PID;

float pidUpdate(float target, float current, PID * pid_state);

#endif /* INC_PID_H_ */

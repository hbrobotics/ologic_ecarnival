/*
 * pid.c
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#include <stdio.h>
#include "pid.h"


// implement basic parallel PID (PI) controller
float pidUpdate(float target, float current, PID * pid)  {

	float error = target - current; // compute error

	PID_STATE * pid_state = &pid->state;

	// compute integral
    float I = pid_state->I + error*pid->dt;

    // reset integral when stopped
    if(target==0.0 && current==0.0) {
    	I=0;
    }

    // compute output as Kp * error + Ki * dT * Integral(error)
	float duty = pid->kp * error + pid->ki * I;

	if(pid->openLoop) {
		duty= target;
	}

	// clamp output to +-1
	if (duty > 1.0) {
		duty = 1.0;
	}

	if (duty < -1.0) {
		duty = -1.0;
	}

	// Long fom message for debugging
	//printf("PID: %s,  target=%d, current=%d, duty=%d, I=%d\n",pid_state->tag,(int)(target*1000),(int)(current*1000),(int)(duty*1000),(int)(I*1000));

	// Short form message for logging and PID tuning in matlab/octave
	//printf("%c,%d,%d,%d,%d\n",pid_state->tag[0],(int)(target*1000),(int)(current*1000),(int)(duty*1000),(int)(I*1000));

	// update statee
	pid_state->error = error;
	pid_state->I = I;

	pid_state->ref=target;
	pid_state->fb=current;
	pid_state->u=duty;

	// return desired output
	return duty;
}

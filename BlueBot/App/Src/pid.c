/*
 * pid.c
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#include <stdio.h>
#include "pid.h"


// implement basic parallel PID (PI) controller
float pidUpdate(float target, float current, PID * pid_state)  {

	float error = target - current; // compute error

	// compute integral
    float I = pid_state->I + error*pid_state->dt;

    // reset integral when stopped
    if(target==0.0 && current==0.0) {
    	I=0;
    }

    // compute output as Kp * error + Ki * dT * Integral(error)
	float duty = pid_state->kp * error + pid_state->ki * I;

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

	// return desired output
	return duty;
}

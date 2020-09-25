/*
 * pid.c
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#include <stdio.h>
#include "pid.h"

float pidUpdate(float target, float current, PID * pid_state)  {

	float error = target - current;
    float I = pid_state->I + error*pid_state->dt;

    if(target==0.0 && current==0.0) {
    	I=0;
    }
	float duty = pid_state->kp * error + pid_state->ki * I;

	if (duty > 1.0) {
		duty = 1.0;
	}

	if (duty < -1.0) {
		duty = -1.0;
	}

	//rintf("PID: %s,  target=%d, current=%d, duty=%d, I=%d\n",pid_state->tag,(int)(target*1000),(int)(current*1000),(int)(duty*1000),(int)(I*1000));
	//printf("%c,%d,%d,%d,%d\n",pid_state->tag[0],(int)(target*1000),(int)(current*1000),(int)(duty*1000),(int)(I*1000));

	pid_state->error = error;
	pid_state->I = I;
	return duty;
}

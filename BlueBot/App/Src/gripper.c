/*
 * gripper.c
 *
 *  Control module for gripper mechanism
 *
 *  Gripper position set by a simple servo - position set by PWM pulse width of servo drive output
 *
 *
 *  Created on: Oct 15, 2020
 *      Author: Ralph Gnauck
 */
#include "tim.h"

#include "gripper.h"


// Set the PWM duty to control the position of the servo
void setGripper(uint32_t pos) {

	// limit duty to lie between full up and full down position
	if(pos < GRIPPER_UP) {
		pos = GRIPPER_UP;
	}

	if(pos > GRIPPER_DOWN) {
		pos = GRIPPER_DOWN;
	}

	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,pos); // set PWM duty

}

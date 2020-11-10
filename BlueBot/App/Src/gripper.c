/*
 * gripper.c
 *
 *  Created on: Oct 15, 2020
 *      Author: ralph
 */
#include "tim.h"

#include "gripper.h"


void setGripper(uint32_t pos) {
	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,pos);

}

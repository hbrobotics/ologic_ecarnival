/*
 * gripper.h
 *
 *  Control module for gripper mechanism
 *
 *  Gripper position set by a simple servo - position set by PWM pulse width of servo drive output
 *
 *  Created on: Oct 15, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

#include <stdint.h>

#define GRIPPER_UP 1000   // PWM duty for UP position
#define GRIPPER_DOWN 1800 // PWM duty for UP position


void setGripper(uint32_t pos); // Set the Gripper position


#endif /* INC_GRIPPER_H_ */

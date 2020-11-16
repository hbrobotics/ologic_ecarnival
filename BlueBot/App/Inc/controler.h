/*
 * controler.h
 *
 *  Implements the main robot state machine controller
 *
 *  Created on: Oct 1, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_CONTROLER_H_
#define INC_CONTROLER_H_


#include "motors.h"
void updateControler(MotorEvent event); // called from main loop to update the state machine


#endif /* INC_CONTROLER_H_ */

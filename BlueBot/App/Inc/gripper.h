/*
 * gripper.h
 *
 *  Created on: Oct 15, 2020
 *      Author: ralph
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

#include <stdint.h>

#define GRIPPER_UP 1000
#define GRIPPER_DOWN 1800


void setGripper(uint32_t pos);


#endif /* INC_GRIPPER_H_ */

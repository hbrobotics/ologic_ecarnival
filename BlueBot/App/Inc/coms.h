/*
 * coms.h
 *
 *  Created on: Oct 15, 2020
 *      Author: ralph
 */

#ifndef INC_COMS_H_
#define INC_COMS_H_

#include <stdio.h>
#include <stdbool.h>
#include "motors.h"

MotorEvent doComs(void);

void slipEncode(uint8_t * buf, int len);
bool slipDecode(uint8_t c, int size, uint8_t * slipInPacket, int * out_len);

#endif /* INC_COMS_H_ */

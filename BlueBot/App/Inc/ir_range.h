/*
 * ir_range.h
 *
 *  Created on: Oct 11, 2020
 *      Author: ralph
 */

#ifndef INC_IR_RANGE_H_
#define INC_IR_RANGE_H_

#define NUM_IR_SENSORS 2

#define LR_IR 0
#define SR_IR 1


void updateIRSensors(void);

float getLongRangeIR(void);
float getShortRangeIR(void);
void checkIRRanges(void);

#endif /* INC_IR_RANGE_H_ */

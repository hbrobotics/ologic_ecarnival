/*
 * ir_range.h
 *
 *  Process distance measurements from teh Sharp IR sensors
 *
 *  Created on: Oct 11, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_IR_RANGE_H_
#define INC_IR_RANGE_H_

#define NUM_IR_SENSORS 2 // Number of sensors

// define sensor IDs
#define LR_IR 0 // Long range sensor
#define SR_IR 1 // short range sensor

// process new sensor readings
void updateIRSensors(void);


float getLongRangeIR(void); // get latest measurement from long range sensor (distance in cm)
float getShortRangeIR(void); // get latest measurement from long range sensor (distance in cm)
void checkIRRanges(void); // print current readings

#endif /* INC_IR_RANGE_H_ */

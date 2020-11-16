/*
 * ir_range.c
 *
 *  Process distance measurements from teh Sharp IR sensors
 *
 *
 *  Created on: Oct 11, 2020
 *      Author: Ralph Gnauck
 */

#include "ir_range.h"
#include "adc_io.h"
#include <stdio.h>
#include <math.h>

// map the ADC readings to the appropreate sensor
#define LR_ADC ADC_1
#define SR_ADC ADC_2


#define SCALE 0.0008056640625f // ADC to volts

// Define calibration coefficients for each sensor - raw data has been fit to a 2nd order polynomial approximation function for calibration
// Coefficients for short range sensor
#define SR_A 15.5f
#define SR_B -0.8647f
#define SR_C -2.494f

// Coefficients for long range sensor
#define LR_A 76.98f
#define LR_B -0.9005f
#define LR_C -13.04f


// current distance reading for each sensor
static float dist[NUM_IR_SENSORS]={-1.0f,-1.0f}; // distance in cm

// calculate calibrated distance from raw reading
static float calibrate(float v,float a,float b, float c) ;


// update current readings if new ADC values are avaialble
// called from the main loop
void updateIRSensors(void) {

	uint32_t value;
	if(get_adc(LR_ADC,&value)) { // get new ADC reading for long range sensor (if any)
		dist[LR_IR]=calibrate(value*SCALE, LR_A, LR_B, LR_C); // calculate distance from raw ADC value
	}
	if(get_adc(SR_ADC,&value)) { // get new ADC reading for short range sensor (if any)
		dist[SR_IR]=calibrate(value*SCALE, SR_A, SR_B, SR_C); // calculate distance from raw ADC value
	}
}

// return latest measurement from long range sensor
// returns distance in cm
float getLongRangeIR(void) {
	return dist[LR_IR];
}

// return latest measurement from short range sensor
// returns distance in cm
float getShortRangeIR(void) {
	return dist[SR_IR];
}

// print current sensor values
void checkIRRanges(void) {

float lr=getLongRangeIR();
float sr=getShortRangeIR();

	//printf("IR: Short=%6.3fcm,  Long=%6.3fcm\n",sr,lr);

}

// calculate calibrated distance from raw reading
// v : raw reading
// a,b,c : calibration coefficients for the sensor type
float calibrate(float v, float a, float b, float c) {
	return a * powf(v,b) +c;
}

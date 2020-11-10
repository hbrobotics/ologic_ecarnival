/*
 * ir_range.c
 *
 *  Created on: Oct 11, 2020
 *      Author: ralph
 */

#include "ir_range.h"
#include "adc_io.h"
#include <stdio.h>
#include <math.h>

#define LR_ADC ADC_1
#define SR_ADC ADC_2

#define SCALE 0.0008056640625 // ADC to volts


#define SR_A 15.5
#define SR_B -0.8647
#define SR_C -2.494

#define LR_A 76.98
#define LR_B -0.9005
#define LR_C -13.04


static float dist[NUM_IR_SENSORS]={-1.0,-1.0};


static float calibrate(float v,float a,float b, float c) ;

void updateIRSensors(void) {
	uint32_t value;
	if(get_adc(LR_ADC,&value)) {
		dist[LR_IR]=calibrate(value*SCALE, LR_A, LR_B, LR_C);
	}
	if(get_adc(SR_ADC,&value)) {
		dist[SR_IR]=calibrate(value*SCALE, SR_A, SR_B, SR_C);
	}
}

float getLongRangeIR(void) {
	return dist[LR_IR];
}

float getShortRangeIR(void) {
	return dist[SR_IR];
}

void checkIRRanges(void) {

float lr=getLongRangeIR();
float sr=getShortRangeIR();

	//printf("IR: Short=%6.3fcm,  Long=%6.3fcm\n",sr,lr);

}

float calibrate(float v, float a, float b, float c) {

	return a * pow(v,b) +c;
}

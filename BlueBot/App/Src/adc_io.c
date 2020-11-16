/*
 * adc_io.c
 *
 *  Read Data From The ADCs
 *
 *  Created on: Oct 11, 2020
 *      Author: Ralph Gnauck
 */

#include "adc.h"
#include "tim.h"
#include "adc_io.h"

// buffer to save data
static uint32_t adc[NUM_ADC];
static volatile bool adc_update[NUM_ADC]={false,false}; // flags set by ISR when new readings are stored in teh buffers


// see if new value ready for an ADC
// id : index of the ADC reading to check
// value : pointer to variable to store new data (if new data is valid)
// returns true if new value was stored in value, else false;
bool get_adc(uint32_t id, uint32_t *value) {

	if(adc_update[id]) { // see if new data flag set for selected reading
		*value=adc[id];  // save the value in the output parameter
		adc_update[id]=false; // reset flag
		return true; // return true to tell caller they got new value
	}
	return false; // no new data
}


// Setup the ADCs, we use both ADCs and take one reading from each. New readings are triggered by timer 6
void adc_init(void) {

	// run STM ADC calibration
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	// start both ADCs in interrupt mode
	// ADCs configured in CubeUI to be triggered from Timer 6
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);

	HAL_TIM_Base_Start(&htim6); // star tthe timer to trigger ADC readings
}


// ISR callback when new ADC readings ready
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {


	// get which ADC is ready
	uint32_t idx =(hadc== &hadc1)?ADC_1:ADC_2;

	adc[idx]=HAL_ADC_GetValue(hadc); // store the value in the buffer
	adc_update[idx]= true; //  set flag to say new data is ready

}

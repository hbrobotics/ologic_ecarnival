/*
 * adc_io.c
 *
 *  Created on: Oct 11, 2020
 *      Author: ralph
 */

#include "adc.h"
#include "tim.h"
#include "adc_io.h"

static uint32_t adc[NUM_ADC];
static bool adc_update[NUM_ADC]={false,false};


bool get_adc(uint32_t id, uint32_t * value) {
	if(adc_update[id]) {
		*value=adc[id];
		adc_update[id]=false;
		return true;
	}
	return false;
}

void adc_init(void) {

	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);

	HAL_TIM_Base_Start(&htim6);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	uint32_t idx =(hadc== &hadc1)?ADC_1:ADC_2;

	adc[idx]=HAL_ADC_GetValue(hadc);
	adc_update[idx]= true;

}

/*
 * adc_io.h
 *
 *  Created on: Oct 11, 2020
 *      Author: ralph
 */

#ifndef INC_ADC_IO_H_
#define INC_ADC_IO_H_
#include <stdint.h>
#include <stdbool.h>

#define NUM_ADC 2

#define ADC_1 0
#define ADC_2 1

void adc_init(void);
bool get_adc(uint32_t id, uint32_t * value);

#endif /* INC_ADC_IO_H_ */

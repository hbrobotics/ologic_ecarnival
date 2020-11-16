/*
 * adc_io.h
 *
 * Read Data From The ADCs
 *
 *  Created on: Oct 11, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_ADC_IO_H_
#define INC_ADC_IO_H_
#include <stdint.h>
#include <stdbool.h>

#define NUM_ADC 2 // Number of ADC readings

// Indexes of ADC readings in buffer
#define ADC_1 0
#define ADC_2 1

void adc_init(void); // setup the ADC
bool get_adc(uint32_t id, uint32_t * value); // get a value from the ADC (returns true if new data is returned,else false)

#endif /* INC_ADC_IO_H_ */

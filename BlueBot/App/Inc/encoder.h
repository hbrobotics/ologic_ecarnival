/*
 * encoder.h
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include "tim.h"

typedef struct ENC_STATUS_t {
	int32_t pos;
	int32_t vel;
	int16_t last;
	const char *tag;
	uint16_t dir;
	const TIM_HandleTypeDef * htim;
} ENC_STATUS;


void updateEncoder(ENC_STATUS * enc);

#endif /* INC_ENCODER_H_ */

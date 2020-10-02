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

#define ENCODER_DIST_SCALE  (1.0/5456.740906) // counts/m
#define ENCODER_VEL_SCALE 0.2617993878     // convert encoder velocity value to rad/sec

// define the Encoder state variables
typedef struct ENC_STATUS_t {
	float pos;     // cumulative position (m), signed relative to 0 when robot starts (wheel going forward increments, backwards decrements position)
	float vel;       // current wheel velocity rad/s.
	int16_t last;    // last position register value, used to calculate differences since last update and detect 16bit timer overflow
	uint16_t dir;    // sets the direction reported by the encoder.  Set to +1 or -1 so encoder gives positive vel. when wheel moves forwards
	const TIM_HandleTypeDef * htim;  // reference to the STM HAL timer used by this encoder

	const char *tag; // Tag (name) of this encoder to show in debug messages
} ENC_STATUS;


void updateEncoder(ENC_STATUS * enc);


extern ENC_STATUS enc_left;
extern ENC_STATUS enc_right;
#endif /* INC_ENCODER_H_ */

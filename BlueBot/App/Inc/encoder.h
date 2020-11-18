/*
 * encoder.h
 *
 *  Process the encoder inputs and calculate position and velocity of each wheel
 *
 *  Created on: Sep 16, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include "tim.h"

#define ENCODER_DIST_SCALE  (1.0f/5456.740906f) // counts/m
#define ENCODER_VEL_SCALE 0.2617993878f     // convert encoder velocity value to rad/sec

// encoder state variables
typedef struct ENCODER_STATE_t {
	float pos;     // cumulative position (m), signed relative to 0 when robot starts (wheel going forward increments, backwards decrements position)
	float vel;     // current wheel velocity rad/s.
} ENCODER_STATE;

// define the Encoder config variables
typedef struct ENC_STATUS_t {

	int16_t last;   // last position register value, used to calculate differences since last update and detect 16bit timer overflow
	float last_vel;
	int16_t dir;    // sets the direction reported by the encoder.  Set to +1 or -1 so encoder gives positive vel. when wheel moves forwards
	const TIM_HandleTypeDef * htim;  // reference to the STM HAL timer used by this encoder

	const char *tag; // Tag (name) of this encoder to show in debug messages

	ENCODER_STATE state; // encoder state info
} ENCODER;


// called at PID update rate to update position and velocity data
void updateEncoder(ENCODER * enc);


// reference to the state for each encoder
extern ENCODER enc_left;
extern ENCODER enc_right;

#endif /* INC_ENCODER_H_ */

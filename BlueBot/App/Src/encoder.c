/*
 * encoder.c
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>

// update encoder state variables with new position and velocity
void updateEncoder(ENC_STATUS * enc) {

	int16_t pos16 = enc->dir*(int16_t) __HAL_TIM_GET_COUNTER(enc->htim); // treat timers as signed 16 bit
	int32_t pos32 = (int32_t)pos16; // sign extend to 32 bit

    int16_t last = enc->last; // get last raw timer value

    int32_t diff = pos32-last; // change in pos (vel)

	if (abs(last) > 20000) { // is timer likely to have over/underflowed (sign changes near 0 are ok)
		if(pos16 < 0 && last >= 0) { // overflow forward
			diff += (int32_t)0x10000;
		}
		else if(pos16 >= 0 && last < 0) { // underflow backwards
			diff -= (int32_t)0x10000;
		}
	}

	// update state
	enc->vel = ENCODER_VEL_SCALE*(float)diff;   // output velocity as rad/sec
	enc->pos += diff*ENCODER_DIST_SCALE;  // position is integral of raw velocity

	// output debug messages
	//printf("Enc %s: pos=%5.2f, vel=%5.2f last=%d\r\n",enc->tag,enc->pos,enc->vel,enc->last);

	enc->last = pos16; // save counter value for next time so we can calculate differences

}

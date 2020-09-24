/*
 * encoder.c
 *
 *  Created on: Sep 16, 2020
 *      Author: ralph
 */

#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>

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

	enc->vel = diff;
	enc->pos += diff;

	//printf("Enc %s: pos=%ld, vel=%ld last=%d\r\n",enc->tag,enc->pos,enc->vel,enc->last);

	enc->last = pos16;

}

/*
 * app_main.c
 *
 *  Created on: Sep 2, 2020
 *      Author: ralph
 */

#include <stdio.h>

#include "app_main.h"
#include "gpio.h"
#include "tim.h"
#include "pid.h"
#include "encoder.h"

#include "main.h"

#define TICK_RATE 10 // 10ms

#define LED_BLINK_RATE 50 // 50*10ms = 1/2 second
#define PID_RATE       2 // 2*10ms = 0.02 second

#define DT ((float)(TICK_RATE*PID_RATE)/1000)

#define KP 0.065
#define KI 2.0


#define MAX_SPEED 14.0  //rad/s

#define ENC_VEL_SCALE 0.2617993878

#define SPEED_CHANGE 0.1


static PID pid_right = {KP,KI,0,0,"Right",DT};
static PID pid_left  = {KP,KI,0,0,"Left",DT};

static ENC_STATUS enc_right = {0,0,0,"Right",1,&htim1};
static ENC_STATUS enc_left  = {0,0,0,"Left",-1,&htim2};


static void setMtrSpeed(uint32_t ch_a, uint32_t ch_b, int16_t speed);



void app_main(void) {

	float speed_l=0;
	float speed_r=0;

	float duty_l=0;
	float duty_r=0;

	uint32_t ledTimer=LED_BLINK_RATE;
	uint32_t pidTimer=PID_RATE;

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);


	printf("E-carnival ready\r\n");

	uint32_t tick = HAL_GetTick();


	while(1) {
		uint32_t tock = HAL_GetTick();
		if(tock-tick > TICK_RATE) { // 10ms timer

			ledTimer--;
			if(ledTimer==0) {
				ledTimer=LED_BLINK_RATE;
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			}

			pidTimer --;
			if(pidTimer==0) {
				pidTimer=PID_RATE;

				updateEncoder(&enc_left);
				updateEncoder(&enc_right);

				duty_l = pidUpdate(speed_l,(float)enc_left.vel*ENC_VEL_SCALE,&pid_left);
				duty_r = pidUpdate(speed_r,(float)enc_right.vel*ENC_VEL_SCALE,&pid_right);

				//duty_l = 0.5;
				//duty_r = 0.5;

				setMtrSpeed(TIM_CHANNEL_1,TIM_CHANNEL_2,duty_l*MTR_PWM_PERIOD);
				setMtrSpeed(TIM_CHANNEL_4,TIM_CHANNEL_3,duty_r*MTR_PWM_PERIOD);
			}

			tick=tock;
		}


		int c = getchar();
		if(c != EOF) {
			putchar(c);

			if(c=='+') {
			  if(speed_l < MAX_SPEED) {
				 speed_l+=SPEED_CHANGE;
			  }
			}

			if(c=='-') {
			  if(speed_l > -MAX_SPEED) {
				 speed_l-=SPEED_CHANGE;
			  }
			}

			if(c=='>') {
			  if(speed_r < MAX_SPEED) {
				 speed_r+=SPEED_CHANGE;
			  }
			}

			if(c=='<') {
			  if(speed_r > -MAX_SPEED) {
				 speed_r-=SPEED_CHANGE;
			  }
			}

			if(c==' ') {
			  speed_l = 0;
			  speed_r = 0;
			}

			if(c=='t') {
				speed_r=MAX_SPEED/2;
			}

		}
		else{
		    clearerr(stdin);
		}

	}

}

void setMtrSpeed(uint32_t ch_a, uint32_t ch_b, int16_t speed) {

	uint16_t duty_a=0;
	uint16_t duty_b=0;

	if(speed == 0) {
	  duty_a=0;
	  duty_b=0;
	}
	else if(speed > 0) {
	  duty_a=speed;
	  duty_b=0;
	}
	else {
	  duty_b=-speed;
	  duty_a=0;
	}
	__HAL_TIM_SET_COMPARE(&htim3,ch_a,duty_a);
	__HAL_TIM_SET_COMPARE(&htim3,ch_b,duty_b);
}


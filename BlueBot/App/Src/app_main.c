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
#define PID_RATE       2 // 2*10ms = 0.02 second (50Hz)

#define DT ((float)(TICK_RATE*PID_RATE)/1000) // compute sample time for PID in seconds

// PID Tunings
#define KP 0.065
#define KI 2.0


#define MAX_SPEED 14.0  //rad/s

#define ENC_VEL_SCALE 0.2617993878 // convert encoder velocity value to rad/sec

#define SPEED_CHANGE 0.1 // input speed step for manual wheel speed commands


// declare the PID state variables
static PID pid_right = {KP,KI,0,0,DT,"Right"};
static PID pid_left  = {KP,KI,0,0,DT,"Left"};

// declare the encoder state variables
static ENC_STATUS enc_right = {0,0,0,1,ENC_VEL_SCALE,&htim1,"Right"};
static ENC_STATUS enc_left  = {0,0,0,-1,ENC_VEL_SCALE,&htim2,"Left"};


// local function prototypes
static void setMtrSpeed(uint32_t ch_a, uint32_t ch_b, float duty);


// main app loop - runs forever
void app_main(void) {

	float speed_l=0; // desired left wheel speed (rad/sec)
	float speed_r=0; // desired right wheel speed (rad/sec)

	float duty_l=0; // left wheel output duty cycle  (-1.0 -- 1.0)
	float duty_r=0; // right wheel output duty cycle (-1.0 -- 1.0)

	uint32_t ledTimer=LED_BLINK_RATE;
	uint32_t pidTimer=PID_RATE;

	// start the PWM outputs
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	// Start the encoder input timers
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);


	printf("E-Carnival Robot Ready\r\n");

	uint32_t tick = HAL_GetTick();


	while(1) {
		uint32_t tock = HAL_GetTick();
		if(tock-tick > TICK_RATE) { // 10ms timer

			ledTimer--; // blink LED once per second
			if(ledTimer==0) {
				ledTimer=LED_BLINK_RATE;
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			}

			pidTimer --;
			if(pidTimer==0) { // Run PID/speed control at set rate (50Hz)
				pidTimer=PID_RATE;

				// get latest speed and position estimates from encoders
				updateEncoder(&enc_left);
				updateEncoder(&enc_right);

				// run PID for speed control
				duty_l = pidUpdate(speed_l,enc_left.vel,&pid_left);
				duty_r = pidUpdate(speed_r,enc_right.vel,&pid_right);

				//duty_l = 0.5;
				//duty_r = 0.5;

				// set output PWM duty for both motors
				setMtrSpeed(TIM_CHANNEL_1,TIM_CHANNEL_2,duty_l);
				setMtrSpeed(TIM_CHANNEL_4,TIM_CHANNEL_3,duty_r);
			}

			tick=tock;
		}

        // process input commands from serial
		int c = getchar();
		if(c != EOF) {
			putchar(c);

			// debug commands to manually control wheel speed
			if(c=='+') { // accelerate left wheel
			  if(speed_l < MAX_SPEED) {
				 speed_l+=SPEED_CHANGE;
			  }
			}

			if(c=='-') { // decelerate left wheel
			  if(speed_l > -MAX_SPEED) {
				 speed_l-=SPEED_CHANGE;
			  }
			}

			if(c=='>') { // accelerate right wheel
			  if(speed_r < MAX_SPEED) {
				 speed_r+=SPEED_CHANGE;
			  }
			}

			if(c=='<') { // decelerate right wheel
			  if(speed_r > -MAX_SPEED) {
				 speed_r-=SPEED_CHANGE;
			  }
			}

			if(c==' ') { // stop both motors
			  speed_l = 0;
			  speed_r = 0;
			}

			if(c=='t') { // generate step response for PID tuning (step right wheel to 50% power)
				speed_r=MAX_SPEED/2;
			}

			if(c=='d') { // drive both wheels forward at 1/10 max speed
				speed_l=MAX_SPEED/10;
				speed_r=MAX_SPEED/10;
			}

			if(c=='b') {  // drive both wheels backward at 1/10 max speed
				speed_l=-MAX_SPEED/10;
				speed_r=-MAX_SPEED/10;
			}

			if(c=='s') {  //  turn (rotate) left at 1/15 max speed
				speed_l=MAX_SPEED/15;
				speed_r=-MAX_SPEED/15;
			}
			if(c=='a') {  //  turn (rotate) right at 1/15 max speed
				speed_l=-MAX_SPEED/15;
				speed_r=MAX_SPEED/15;
			}
			clearerr(stdin);
		}
		else{
		    clearerr(stdin);
		}

	}

}


//
// Set PWM output for a motor for desired power
//
// calculate PWM outputs for the Gate driver A,B outputs
// Uses the sign of desired duty to set A or B outputs as 0 or PWM signals
// Duty of active PWM is abs(duty)
//
// duty:  -1 >= duty <= 1
void setMtrSpeed(uint32_t ch_a, uint32_t ch_b, float duty) {

	uint16_t duty_a=0;
	uint16_t duty_b=0;

	duty *= MTR_PWM_PERIOD; // scale to get proper value for duty

	if(duty == 0.0) { // Stopped A,B=0
	  duty_a=0;
	  duty_b=0;
	}
	else if(duty > 0.0) { // going forward A is PWM, B=0
	  duty_a=(uint16_t)duty;
	  duty_b=0;
	}
	else {
	  duty_b=(uint16_t)-duty; // going backwards A=0, B is PWM
	  duty_a=0;
	}

	// output desired duty for the A,B PWM outputs
	__HAL_TIM_SET_COMPARE(&htim3,ch_a,duty_a);
	__HAL_TIM_SET_COMPARE(&htim3,ch_b,duty_b);
}


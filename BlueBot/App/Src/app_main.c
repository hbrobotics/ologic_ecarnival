/*
 * app_main.c
 *
 *  Created on: Sep 2, 2020
 *      Author: ralph
 */

#include <stdio.h>
#include <math.h>

#include "app_main.h"
#include "gpio.h"
#include "tim.h"
#include "pid.h"
#include "encoder.h"
#include "edge_sensor.h"
#include "main.h"
#include "motors.h"
#include "controler.h"


#define TICK_RATE 10 // 10msd

#define LED_BLINK_RATE 50 // 50*10ms = 1/2 second
#define PID_RATE       2 // 2*10ms = 0.02 second (50Hz)

#define DT ((float)(TICK_RATE*PID_RATE)/1000) // compute sample time for PID in seconds

// PID Tunings
#define KP 0.065
#define KI 2.0


#define SPEED_CHANGE 0.1 // input speed step for manual wheel speed commands

#define GRIPPER_UP 1000
#define GRIPPER_DOWN 1800

// declare the PID state variables
PID pid_right = {KP,KI,0,0,DT,"Right"};
PID pid_left  = {KP,KI,0,0,DT,"Left"};

// declare the encoder state variables
ENC_STATUS enc_right = {0,0,0,1,&htim1,"Right"};
ENC_STATUS enc_left  = {0,0,0,-1,&htim2,"Left"};


// local function prototypes
static void setGripper(uint32_t pos);

// main app loop - runs forever
void app_main(void) {



	uint32_t ledTimer=LED_BLINK_RATE;
	uint32_t pidTimer=PID_RATE;

	// start the PWM outputs
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1); // gripper PWM

	// Start the encoder input timers
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);


	printf("E-Carnival Robot Ready\r\n");

	uint32_t tick = HAL_GetTick();

	setGripper(GRIPPER_UP);
	enableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);

	while(1) {
		uint32_t tock = HAL_GetTick();

		bool pid_update=false;

		if(tock-tick > TICK_RATE) { // 10ms timer

			ledTimer--; // blink LED once per second
			if(ledTimer==0) {
				ledTimer=LED_BLINK_RATE;
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			}

			pidTimer --;
			if(pidTimer==0) { // Run PID/speed control at set rate (50Hz)
				pidTimer=PID_RATE;
				pid_update=true;

			}

			updateEdgeSensors(); // update debounced states of edge sensors

			tick=tock;
		}

		MotorEvent event =updateMotors(pid_update,DT);

        // process input commands from serial
		int c = getchar();
		if(c != EOF) {
			putchar(c);
#if 0
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
#endif
			if(c==' ') { // stop both motors
			    STOP();
			    event |= ME_STOP;
			}

			//if(c=='t') { // generate step response for PID tuning (step right wheel to 50% power)
				//speed_r=MAX_SPEED/2;
			//}

			if(c=='w') { // drive both wheels forward at 1/10 max speed
				drive(MAX_LIN_VEL/2,0);
			}

			if(c=='z') {  // drive both wheels backward at 1/10 max speed
				drive(-MAX_LIN_VEL/2,0);
			}

			if(c=='s') {  //  turn (rotate) left at 1/15 max speed
				drive(0,-MAX_ANG_VEL/4);

			}
			if(c=='a') {  //  turn (rotate) right at 1/15 max speed
				drive(0,MAX_ANG_VEL/4);
			}

			if(c=='l') {  //  turn (rotate) right at 1/15 max speed
				turnTo(M_PI/4.0,MAX_ANG_VEL/4);
			}

			if(c=='r') {  //  turn (rotate) right at 1/15 max speed
				turnTo(-M_PI/4.0,MAX_ANG_VEL/4);
			}

			if(c=='f') {  //  turn (rotate) right at 1/15 max speed
				driveTo(0.3,MAX_LIN_VEL/4);
			}

			if(c=='b') {  //  turn (rotate) right at 1/15 max speed
				driveTo(-0.3,MAX_LIN_VEL/4);
			}

			if(c=='G') {  //  move gripper up
				setGripper(GRIPPER_UP);
			}

			if(c=='g') {  // move gripper down
				setGripper(GRIPPER_DOWN);
			}

			if(c=='1') {
				event |= CE_M1;
			}

			if(c=='2') {
				event |= CE_M2;
			}

			if(c=='3') {
				event |= CE_M3;
			}

			clearerr(stdin);
		}
		else{
		    clearerr(stdin);
		}

		updateControler(event);

	}

}




void setGripper(uint32_t pos) {
	__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,pos);

}


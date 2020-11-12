/*
 * app_main.c
 *
 *  Created on: Sep 2, 2020
 *      Author: ralph
 */

#include <stdio.h>
#include <stdbool.h>
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
#include "adc_io.h"
#include "ir_range.h"
#include "coms.h"
#include "ui.h"
#include "gripper.h"



#define TICK_RATE 10 // 10msd

#define LED_BLINK_RATE 50 // 50*10ms = 1/2 second
#define PID_RATE       2 // 2*10ms = 0.02 second (50Hz)

#define DT (((float)(TICK_RATE*PID_RATE))/1000.0f) // compute sample time for PID in seconds

// PID Tunings
#define KP 0.067f // 0.1064// 0.065
#define KI 1.0f // 0.1242 //2.0




// declare the PID state variables
PID pid_right = {KP,KI,DT,false,"Right", {0.0f,0.0f,0.0f,0.0f,0.0f}};
PID pid_left  = {KP,KI,DT,false,"Left", {0.0f,0.0f,0.0f,0.0f,0.0f}};

// declare the encoder state variables
ENCODER enc_right = {0,0.0f,1,&htim1,"Right",{0.0f,0.0f}};
ENCODER enc_left  = {0,0.0f,-1,&htim2,"Left",{0.0f,0.0f}};




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


	//printf("E-Carnival Robot Ready\r\n");

	uint32_t tick = HAL_GetTick();

	setGripper(GRIPPER_UP);
	enableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);
	adc_init();

	while(1) {
		uint32_t tock = HAL_GetTick();

		bool pid_update=false;
		bool send_telemetry=false;

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
                send_telemetry=true;

			}

			updateEdgeSensors(); // update debounced states of edge sensors


			tick=tock;
		}

		updateIRSensors();
		setIRRangeState(getLongRangeIR(),getShortRangeIR());

		MotorEvent event = updateMotors(pid_update,DT);
		if(pid_update) {
			setPIDState(&pid_left.state,&pid_right.state);
			setEncoderState(&enc_left.state,&enc_right.state);
		}

		event |= doComs();

		//bool leftClif = getEdgeSensorState(BUMP_BIT_LEFT)==ES_HIT;
		//bool rightClif= getEdgeSensorState(BUMP_BIT_RIGHT)==ES_HIT;

		////if(leftClif || rightClif) {
		//	STOP();
		//	event |= leftClif?ME_BUMP_LEFT:ME_BUMP_RIGHT;
		//}
		updateControler(event);

		if(send_telemetry) {
			sendTelemetry();
		}

	}

}





/*
 * app_main.c
 *
 *  Main application module
 *
 *  Created on: Sep 2, 2020
 *      Author: Ralph Gnauck
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



#define TICK_RATE 10 // main timer resolution 10ms

#define LED_BLINK_RATE 50 // 50*10ms = 1/2 second
#define PID_RATE       2  // 2*10ms = 0.02 second (50Hz)

#define DT (((float)(TICK_RATE*PID_RATE))/1000.0f) // compute sample time for PID in seconds

// PID Tunings
#define KP 0.067f // 0.1064// 0.065
#define KI 1.0f   // 0.1242 //2.0




// declare the PID state variables
PID pid_right = {KP,KI,DT,false,"Right", {0.0f,0.0f,0.0f,0.0f,0.0f}};
PID pid_left  = {KP,KI,DT,false,"Left", {0.0f,0.0f,0.0f,0.0f,0.0f}};

// declare the encoder state variables
ENCODER enc_right = {0,0.0f,1,&htim1,"Right",{0.0f,0.0f}};
ENCODER enc_left  = {0,0.0f,-1,&htim2,"Left",{0.0f,0.0f}};




// main app loop - runs forever
void app_main(void) {


    // Setup timers for blinky LED and running the motor control PID loop
	uint32_t ledTimer=LED_BLINK_RATE;
	uint32_t pidTimer=PID_RATE;

	// start the PWM outputs
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1); // start gripper PWM

	// Start the encoder input timers
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);


	//printf("E-Carnival Robot Ready\r\n");

	uint32_t tick = HAL_GetTick(); // init the main timer, timing based on HAL_TICKS (1ms) intervals

	setGripper(GRIPPER_UP); // start with gripper in the up position

	enableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT); // enable the edge/drop sensors so we don't go over the edge of the table
	adc_init(); // start the ADC for the IR range sensors

	// now do this forever
	while(1) {

		uint32_t tock = HAL_GetTick(); // get timer ticks (1ms per tick)


		bool pid_update=false;     // flag to say if we should update the PID this time through the loop
		bool send_telemetry=false; // flag to say if we should send updated telemetry data to host this time through the loop

		if(tock-tick > TICK_RATE) { // 10ms timer (this 'if' is true once every 10ms)

			ledTimer--; // blink LED at LED_BLINK_RATE
			if(ledTimer==0) {
				ledTimer=LED_BLINK_RATE;
				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

			}

			pidTimer --; // see if we should run the PID update this time through the loop
			if(pidTimer==0) {
				pidTimer=PID_RATE; //
				pid_update=true;     // flag to update PID this time
                send_telemetry=true; // also send new telemetry after we update the PID

			}

			updateEdgeSensors(); // update de-bounced states of edge sensors (run debounce filter at 10ms rate)


			tick=tock; // update main timer for next period
		}

		updateIRSensors(); // update the IR sensor readings

		setIRRangeState(getLongRangeIR(),getShortRangeIR()); // save IR values to telemetry

		// update the motor controller state (handles driving to distance/turns etc)
		// will also update the PID controller if the flag is set
		MotorEvent event = updateMotors(pid_update,DT); // returns events flags if state changed or edge sensor triggered etc


		if(pid_update) {  // if we updated the PID this time round  then update the telemetry with new STATE of PID and encoders
			setPIDState(&pid_left.state,&pid_right.state);
			setEncoderState(&enc_left.state,&enc_right.state);
		}

		event |= doComs(); // process the input UART and get any events raised by the UI

		updateControler(event); // update the main state machine (giving it any events that should be handled)

		if(send_telemetry) { // if flag is set send new telemetry data to the host
			sendTelemetry();
		}

	}

}





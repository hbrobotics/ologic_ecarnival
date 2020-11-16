/*
 * ui.c
 *
 *  User interface module
 *
 *  Receives input commands and transmits robot status to a client
 *
 *  Created on: Oct 15, 2020
 *      Author: Ralph Gnauck
 */

#include <stdio.h>
#include "gripper.h"
#include "ui.h"
#include "motors.h"
#include "coms.h"
#include <stdlib.h>
#include "main.h"

#define MAX_RAND_SPEED 0.5f // Max speed for random step command generation (for PID Tuning)

// Data structure for sending robot state (telemetry) over the coms/wireless link to a host PC
typedef struct telemetry_t {

  /// PID State
  PID_STATE pid_left;
  PID_STATE pid_right;

  // Encoder Status
  ENCODER_STATE enc_left;
  ENCODER_STATE enc_right;

  // IR ranges
  float ir_range_short;
  float ir_range_long;

  uint32_t clifs; // Bump sensor states

} TELEMETRY;

static TELEMETRY telemetry; // Variable to hold the current telementry status


static float randf(float max); // generate random float (0.0-max)


#define SPEED_CHANGE 0.1f // input speed step for manual wheel speed commands


// called from main loop to process UI commands
//
// packet :  pointer to input command received from UART (only first char used for now)
// len    :  length of input packet
// event  : pointer to MotorEvent to return to be sent to the Controller State Machine
void doUI(uint8_t * packet, int len, MotorEvent *event) {


	uint8_t c=packet[0]; // just use first character in input packet as the command

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
			*event |= ME_STOP;
		}

		if(c=='t') { // generate step command for PID tuning (open loop) (step  wheels to random power)
			float v = randf(MAX_RAND_SPEED);
			setMotorSpeed(v,v);
		}

		if(c=='T') { // generate reverse step command for PID tuning (open loop) (step wheels to random -power)
			float v = randf(MAX_RAND_SPEED);
			setMotorSpeed(-v,-v);

		}

		if(c=='p') { // generate turning step command for PID tuning (open loop) (step  wheels to random power)
			float v = randf(MAX_RAND_SPEED);

			setMotorSpeed(-v,v);
		}

		if(c=='P') { // generate reverse turning step command for PID tuning (open loop) (step  wheels to random power)
			float v = randf(MAX_RAND_SPEED);
			setMotorSpeed(v,-v);
		}

		if(c=='o') { // put PID in closed loop mode
			setOpenLoop(&pid_left,false);
			setOpenLoop(&pid_right,false);
			//printf("Motors In Closed Loop Mode\n");
		}

		if(c=='O') { // put PID in open loop mode (bypass PID)
			setOpenLoop(&pid_left,true);
			setOpenLoop(&pid_right,true);
			//printf("Motors In Open Loop Mode\n");
		}

		if(c=='w') { // drive both wheels forward at 1/2 max speed
			drive(MAX_LIN_VEL/2.0f,0.0f);
		}

		if(c=='z') {  // drive both wheels backward at 1/2 max speed
			drive(-MAX_LIN_VEL/2.0f,0.0f);
		}

		if(c=='s') {  //  turn (rotate) left at 1/4 max speed
			drive(0.0f,-MAX_ANG_VEL/4.0f);

		}
		if(c=='a') {  //  turn (rotate) right at 1/4 max speed
			drive(0.0f,MAX_ANG_VEL/4.0f);
		}

		if(c=='l') {  //  turn to (rotate 45 Deg) left at 1/4 max speed
			turnTo(M_PI/4.0f,MAX_ANG_VEL/4.0f);
		}

		if(c=='r') {  //  turn to (rotate 45 Deg) right at 1/4 max speed
			turnTo(-M_PI/4.0f,MAX_ANG_VEL/4.0f);
		}

		if(c=='f') {  //  Drive forwards 300mm @ 1/4 max speed
			driveTo(0.3f,MAX_LIN_VEL/4.0f);
		}

		if(c=='b') {  //  Drive backwards 300mm @ 1/4 max speed
			driveTo(-0.3f,MAX_LIN_VEL/4.0f);
		}

		if(c=='G') {  //  move gripper up
			setGripper(GRIPPER_UP);
		}

		if(c=='g') {  // move gripper down
			setGripper(GRIPPER_DOWN);
		}

		if(c=='1') { // return event to start controller in table top challenge level 1 mode
			*event |= CE_M1;
		}

		if(c=='2') { // return event to start controller in table top challenge level 2 mode
			*event |= CE_M2;
		}

		if(c=='3') { // return event to start controller in table top challenge level 3 mode
			*event |= CE_M3;
		}



}


// send the current telemetry info out the UART (data encoded in a slip packet)
void sendTelemetry(void) {
	slipEncode((uint8_t*)&telemetry,sizeof(telemetry));
}


// Update the current telemetry encoder state
void setEncoderState(ENCODER_STATE * enc_left, ENCODER_STATE * enc_right) {
	telemetry.enc_left = *enc_left;
	telemetry.enc_right = *enc_right;
}

// Update the current telemetry PID state
void setPIDState(PID_STATE * pid_left, PID_STATE * pid_right) {
	telemetry.pid_left = *pid_left;
	telemetry.pid_right = *pid_right;
}

// Update the current telemetry motor state info
void setMotorState(void) {
   // TBD
}

// Update the current telemetry controller state info
void setControlerState(void) {
   // TBD
}

// Update IR ranges in the current telemetry
void setIRRangeState(float range_long, float range_short) {
	telemetry.ir_range_long=range_long;
	telemetry.ir_range_short=range_short;
}

// Generate random float from 0-max
float randf(float max) {
	return max *  ((float)rand())/((float)RAND_MAX);
}

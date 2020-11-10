/*
 * ui.c
 *
 *  Created on: Oct 15, 2020
 *      Author: ralph
 */

#include <stdio.h>
#include "gripper.h"
#include "ui.h"
#include "motors.h"
#include "coms.h"
#include <stdlib.h>
#include "main.h"

typedef struct telemetry_t {
  PID_STATE pid_left;
  PID_STATE pid_right;

  ENCODER_STATE enc_left;
  ENCODER_STATE enc_right;

  float ir_range_short;
  float ir_range_long;

  uint32_t clifs;

} TELEMETRY;

static TELEMETRY telemetry;

static float randf(void);

#define SPEED_CHANGE 0.1 // input speed step for manual wheel speed commands


void doUI(uint8_t * packet, int len, MotorEvent *event) {


	uint8_t c=packet[0];

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

		if(c=='t') { // generate step response for PID tuning (step  wheels to 50% power)
			float v = randf();

			setMotorSpeed(v,v);
		}

		if(c=='T') { // generate reverse step response for PID tuning (step  wheels to 50% power)
			float v = randf();
			setMotorSpeed(-v,-v);

		}

		if(c=='p') { // generate step response for PID tuning (step  wheels to 50% power)
			float v = randf();

			setMotorSpeed(-v,v);
		}

		if(c=='P') { // generate reverse step response for PID tuning (step  wheels to 50% power)
			float v = randf();
			setMotorSpeed(v,-v);

		}

		if(c=='o') {
			setOpenLoop(&pid_left,false);
			setOpenLoop(&pid_right,false);
			printf("Motors In Closed Loop Mode\n");
		}

		if(c=='O') {
			setOpenLoop(&pid_left,true);
			setOpenLoop(&pid_right,true);
			printf("Motors In Open Loop Mode\n");
		}

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
			*event |= CE_M1;
		}

		if(c=='2') {
			*event |= CE_M2;
		}

		if(c=='3') {
			*event |= CE_M3;
		}



}

void sendTelemetry(void) {
	slipEncode((uint8_t*)&telemetry,sizeof(telemetry));
}

void setEncoderState(ENCODER_STATE * enc_left, ENCODER_STATE * enc_right) {
	telemetry.enc_left = *enc_left;
	telemetry.enc_right = *enc_right;
}

void setPIDState(PID_STATE * pid_left, PID_STATE * pid_right) {
	telemetry.pid_left = *pid_left;
	telemetry.pid_right = *pid_right;
}

void setMotorState() {

}

void setControlerState() {

}

void setIRRangeState(float range_long, float range_short) {
	telemetry.ir_range_long=range_long;
	telemetry.ir_range_short=range_short;
}


float randf(void) {
	return 0.5 *  ((float)rand())/((float)RAND_MAX);

}

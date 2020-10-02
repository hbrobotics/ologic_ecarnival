/*
 * motors.c
 *
 *  Created on: Sep 28, 2020
 *      Author: ralph
 */

#include <stdio.h>

#include "main.h"
#include "motors.h"
#include "tim.h"

#include "encoder.h"
#include "pid.h"
#include "edge_sensor.h"


#define WHEEL_BASE   0.087
#define WHEEL_RADIUS (0.070/2.0)

static void setMtrSpeed(uint32_t ch_a, uint32_t ch_b, float duty);
static void updatePose(float DT);

static float speed_l=0; // desired left wheel speed (rad/sec)
static float speed_r=0; // desired right wheel speed (rad/sec)

static float target_dist=0.0;
static float target_heading=0.0;

static float start_pose_x=0.0;
static float start_pose_y=0.0;
static float start_heading=0.0;

float pose_x=0.0;
float pose_y=0.0;
float heading=0.0;

static bool driving=false;
static bool turn_ccw=false;
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

void STOP(void) {
	speed_l = 0.0;
	speed_r = 0.0;
	setMtrSpeed(TIM_CHANNEL_1,TIM_CHANNEL_2,0.0);
	setMtrSpeed(TIM_CHANNEL_4,TIM_CHANNEL_3,0.0);
	driving = false;
}

void drive(float lin_vel, float ang_vel) {
	speed_l =  (lin_vel - ang_vel * WHEEL_BASE/2.0)/WHEEL_RADIUS;
	speed_r =  (lin_vel + ang_vel * WHEEL_BASE/2.0)/WHEEL_RADIUS;
}

MotorEvent updateMotors(bool pid_update, float DT) {

	MotorEvent event = ME_NONE;

	if(pid_update) {
		float duty_l=0; // left wheel output duty cycle  (-1.0 -- 1.0)
		float duty_r=0; // right wheel output duty cycle (-1.0 -- 1.0)

		// get latest speed and position estimates from encoders
		updateEncoder(&enc_left);
		updateEncoder(&enc_right);

		// run PID for speed control
		duty_l = pidUpdate(speed_l,enc_left.vel,&pid_left);
		duty_r = pidUpdate(speed_r,enc_right.vel,&pid_right);

		// set output PWM duty for both motors
		setMtrSpeed(TIM_CHANNEL_1,TIM_CHANNEL_2,duty_l);
		setMtrSpeed(TIM_CHANNEL_4,TIM_CHANNEL_3,duty_r);

		updatePose(DT);

		float ref_heading = heading;

		if(driving && (target_heading != 0.0)) {


			if (((ref_heading < 0.0) && (start_heading >=0.0)) || ((ref_heading >= 0.0) && (start_heading < 0.0))) {
				if(ref_heading < 0 ) {
					if(turn_ccw) {
					   ref_heading += 2.0 * M_PI;
					}
				}
				else {
					if(!turn_ccw) {
					   ref_heading -= 2.0 * M_PI;
					}
				}
			}

			if(fabs(ref_heading-start_heading) >= target_heading) {
				STOP();
				event = ME_DONE_TURN;
			}
		}

		if(driving && (target_dist != 0.0)) {
			if (sqrtf(powf(pose_x-start_pose_x,2.0)+powf(pose_y-start_pose_y,2.0)) >= target_dist) {
				STOP();
				event = ME_DONE_DRIVE;

			}
		}
		if(driving) {
			printf("sh=%5.2f, th=%5.2f, h=%5.2f, rh=%5.2f\n",start_heading,target_heading,heading,ref_heading);
		}

	}

	bool leftClif = getEdgeSensorState(BUMP_BIT_LEFT)==ES_HIT;
	bool rightClif= getEdgeSensorState(BUMP_BIT_RIGHT)==ES_HIT;

	if(leftClif || rightClif) {
		STOP();
		event = leftClif?ME_BUMP_LEFT:ME_BUMP_RIGHT;
	}

	return event;
}

void turnTo(float angle, float ang_vel) {

	start_heading = heading;
	target_heading = fabs(angle);

	target_dist=0.0;
	driving=true;

	if(angle<0.0) {
		drive(0,-fabs(ang_vel));
	}
	else {
		drive(0,fabs(ang_vel));
	}
}

void driveTo(float dist, float lin_vel) {

	start_pose_x = pose_x;
	start_pose_y = pose_y;

	target_dist = fabs(dist);
	target_heading= 0.0;

	driving=true;

	if(dist < 0.0) {
	   turn_ccw=true;
	   drive(-lin_vel,0.0);

	}
	else {
	   turn_ccw=false;
	   drive(lin_vel,0.0);
	}
}

void updatePose(float DT) {

	float dl = enc_left.vel*DT*WHEEL_RADIUS;
	float dr = enc_right.vel*DT*WHEEL_RADIUS;

	float d = (dl+dr)/2.0;
	float dt = (dr-dl)/WHEEL_BASE;

    heading += dt;


	if(heading > M_PI) {
		heading -= 2.0 * M_PI;
	}
	else if(heading <= -M_PI) {
		heading += 2.0 * M_PI;
	}

    pose_x += d * cosf(heading);
	pose_y += d * sinf(heading);

}
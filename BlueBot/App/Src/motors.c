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


#define WHEEL_BASE   0.087f
#define WHEEL_RADIUS (0.070f/2.0f)

const float M_PI_F = (3.141592653589793f);
const float M_2PI_F = (2.0f*3.141592653589793f);

static void setMtrSpeed(uint32_t ch_a, uint32_t ch_b, float duty);
static void updatePose(float DT);

static float speed_l=0.0f; // desired left wheel speed (rad/sec)
static float speed_r=0.0f; // desired right wheel speed (rad/sec)

static float target_dist_2=0.0f;
static float target_heading=0.0f;

static float start_pose_x=0.0f;
static float start_pose_y=0.0f;
static float start_heading=0.0f;

float pose_x=0.0f;
float pose_y=0.0f;
float heading=0.0f;

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

	int16_t duty_i = duty * MTR_PWM_PERIOD; // scale to get proper value for duty

	if(duty_i == 0) { // Stopped A,B=0
	  duty_a=0;
	  duty_b=0;
	}
	else if(duty_i > 0) { // going forward A is PWM, B=0
	  duty_a=duty_i;
	  duty_b=0;
	}
	else {
	  duty_b=-duty_i; // going backwards A=0, B is PWM
	  duty_a=0;
	}

	// output desired duty for the A,B PWM outputs
	__HAL_TIM_SET_COMPARE(&htim3,ch_a,duty_a);
	__HAL_TIM_SET_COMPARE(&htim3,ch_b,duty_b);
}

void STOP(void) {
	speed_l = 0.0f;
	speed_r = 0.0f;
	setMtrSpeed(TIM_CHANNEL_1,TIM_CHANNEL_2,0.0f);
	setMtrSpeed(TIM_CHANNEL_4,TIM_CHANNEL_3,0.0f);
	driving = false;
}

void setMotorSpeed(float left, float right) {
	speed_l = left;
	speed_r = right;
}

void drive(float lin_vel, float ang_vel) {
	speed_l =  (lin_vel - ang_vel * WHEEL_BASE/2.0f)/WHEEL_RADIUS;
	speed_r =  (lin_vel + ang_vel * WHEEL_BASE/2.0f)/WHEEL_RADIUS;
}

MotorEvent updateMotors(bool pid_update, float DT) {

	MotorEvent event = ME_NONE;

	if(pid_update) {
		float duty_l=0.0f; // left wheel output duty cycle  (-1.0 -- 1.0)
		float duty_r=0.0f; // right wheel output duty cycle (-1.0 -- 1.0)

		// get latest speed and position estimates from encoders
		updateEncoder(&enc_left);
		updateEncoder(&enc_right);

		// run PID for speed control
		duty_l = pidUpdate(speed_l,enc_left.state.vel,&pid_left);
		duty_r = pidUpdate(speed_r,enc_right.state.vel,&pid_right);

		// set output PWM duty for both motors
		setMtrSpeed(TIM_CHANNEL_1,TIM_CHANNEL_2,duty_l);
		setMtrSpeed(TIM_CHANNEL_4,TIM_CHANNEL_3,duty_r);

		updatePose(DT);

		float ref_heading = heading;

		if(driving && (target_heading != 0.0f)) {


			if (((ref_heading < 0.0f) && (start_heading >=0.0f)) || ((ref_heading >= 0.0f) && (start_heading < 0.0f))) {
				if(ref_heading < 0.0f ) {
					if(turn_ccw) {
					   ref_heading += M_2PI_F;
					}
				}
				else {
					if(!turn_ccw) {
					   ref_heading -= M_2PI_F;
					}
				}
			}

			if(fabsf(ref_heading-start_heading) >= target_heading) {
				STOP();
				event = ME_DONE_TURN;
			}
		}

		if(driving && (target_dist_2 != 0.0f)) {
			if ((pose_x-start_pose_x*pose_x-start_pose_x+pose_y-start_pose_y*pose_x-start_pose_y) >= target_dist_2) {
				STOP();
				event = ME_DONE_DRIVE;

			}
		}
		if(driving) {
			//printf("sh=%5.2f, th=%5.2f, h=%5.2f, rh=%5.2f\n",start_heading,target_heading,heading,ref_heading);
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
	target_heading = fabsf(angle);

	target_dist_2=0.0f;
	driving=true;

	if(angle<0.0f) {
		drive(0.0f,-fabsf(ang_vel));
	}
	else {
		drive(0.0f,fabsf(ang_vel));
	}
}

void driveTo(float dist, float lin_vel) {

	start_pose_x = pose_x;
	start_pose_y = pose_y;

	target_dist_2 = dist*dist; // use squared distance to save abs and sqrt
	target_heading= 0.0f;

	driving=true;

	if(dist < 0.0f) {
	   turn_ccw=true;
	   drive(-lin_vel,0.0f);

	}
	else {
	   turn_ccw=false;
	   drive(lin_vel,0.0f);
	}
}

void updatePose(float DT) {

	float dl = enc_left.state.vel*DT*WHEEL_RADIUS;
	float dr = enc_right.state.vel*DT*WHEEL_RADIUS;

	float d = (dl+dr)/2.0f;
	float dt = (dr-dl)/WHEEL_BASE;

    heading += dt;


	if(heading > M_PI_F) {
		heading -= M_2PI_F;
	}
	else if(heading <= -M_PI_F) {
		heading += M_2PI_F;
	}

    pose_x += d * cosf(heading);
	pose_y += d * sinf(heading);

}



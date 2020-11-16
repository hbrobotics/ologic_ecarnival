/*
 * motors.c
 *
 *
 *  Implements functions to control the motors and robot travel
 *  Implements forward and reverse kinematics to control robot speed, rotation and calculate a ego centric pose
 *
 *  Created on: Sep 28, 2020
 *      Author: Ralph Gnauck
 */

#include <stdio.h>

#include "main.h"
#include "motors.h"
#include "tim.h"

#include "encoder.h"
#include "pid.h"
#include "edge_sensor.h"

// define robot geometry to calculate kinematics
#define WHEEL_BASE   0.087f              // robot wheel base (distance between the wheels) m
#define WHEEL_RADIUS (0.070f/2.0f)       // radius of the wheels (m)

// define PI and 2*PI as floats
const float M_PI_F = (3.141592653589793f);
const float M_2PI_F = (2.0f*3.141592653589793f);

// local prototypes
static void setMtrSpeed(uint32_t ch_a, uint32_t ch_b, float duty);
static void updatePose(float DT);


static float speed_l=0.0f; // desired left wheel speed (rad/sec)
static float speed_r=0.0f; // desired right wheel speed (rad/sec)

static float target_dist_2=0.0f;  // how far to drive (squared) when running a driveTo command - in meters
static float target_heading=0.0f; // how far to turn when running a turnTo command - in rad

// reference starting pose of robot when beginning a turnTo or driveTo command
static float start_pose_x=0.0f;
static float start_pose_y=0.0f;
static float start_heading=0.0f;

// current robot pose
static float pose_x=0.0f;
static float pose_y=0.0f;
static float heading=0.0f;

// flags to control the driveTo and turnTo commands
static bool driving=false; // true if currently performing a driveTo or turnTo command
static bool turn_ccw=false; // set direction of turn in a turnTo, true = counter clock wise (to the left)

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


// stop both motors and cancel any driveTo or turnTo command that is executing
void STOP(void) {

	// set target speeds to 0
	speed_l = 0.0f;
	speed_r = 0.0f;

	// set PWM output to 0 immediately
	setMtrSpeed(TIM_CHANNEL_1,TIM_CHANNEL_2,0.0f);
	setMtrSpeed(TIM_CHANNEL_4,TIM_CHANNEL_3,0.0f);

	// Cancel driving commands
	driving = false;
}

// set target velocity for each wheel (in rad/s)
void setMotorSpeed(float left, float right) {
	speed_l = left;
	speed_r = right;
}


// set target velocities for each wheel based on desired robot dynamics
// lin_vel : desired linear velocity of robot center (m/s)
// ang_vel : desired angular velocity of robot (rad/s)
void drive(float lin_vel, float ang_vel) {

	// calculate individual wheel speeds from differential drive kinematics equations
	speed_l =  (lin_vel - ang_vel * WHEEL_BASE/2.0f)/WHEEL_RADIUS;
	speed_r =  (lin_vel + ang_vel * WHEEL_BASE/2.0f)/WHEEL_RADIUS;
}

// update the motor controller and robot driving status
// if pid_update is true will update the PID controller also (and associated state like the encoders and output the speeds to the wheels)
// also updates the internal robot pose
// DT is the update period (sec) used for teh inverse kinematics to update the internal pose estimate
//
// Returns any events that are trigered like end of driveTo or turnTo command or is a bump sensor is detected
//
// If at any time the motors are driving and an enabled bumb sensor detects a hit both motors are imediatly stopped.
//
MotorEvent updateMotors(bool pid_update, float DT) {


	MotorEvent event = ME_NONE;

	if(pid_update) { // see if we should update the PID this time through

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

		updatePose(DT); // calculate updated pose

		// now test if we have complted a turn to or driveTo command (if one is running)
		float ref_heading = heading; // get current heading

		if(driving && (target_heading != 0.0f)) { // if doing a turnTo command

            // see if we will turn through 0 heading and handle wrap around of angles if needed
			if (((ref_heading < 0.0f) && (start_heading >=0.0f)) || ((ref_heading >= 0.0f) && (start_heading < 0.0f))) {


				// handle wrapping around target from + to - angles
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

			// now see if we have turned far enough
			if(fabsf(ref_heading-start_heading) >= target_heading) {
				STOP(); // turn completed so stop and refurn event
				event = ME_DONE_TURN;
			}
		}

		// check if doing a driveTo command and stop if we have gone far enough
		if(driving && (target_dist_2 != 0.0f)) {

			// calculate squared magnitude of distance we have moved
			float dx=pose_x-start_pose_x;
			float dy=pose_y-start_pose_y;

			if (( dx*dx+dy*dy) >= target_dist_2) { // compare to square magnatude of target distance
				STOP(); // got htere so stop
				event = ME_DONE_DRIVE; // return done event

			}
		}
		if(driving) {
			//printf("sh=%5.2f, th=%5.2f, h=%5.2f, rh=%5.2f\n",start_heading,target_heading,heading,ref_heading);
		}

	}

	// check if either bumper has a hit (if enabled)
	bool leftClif = getEdgeSensorState(BUMP_BIT_LEFT)==ES_HIT;
	bool rightClif= getEdgeSensorState(BUMP_BIT_RIGHT)==ES_HIT;

	if(leftClif || rightClif) {
		STOP(); // stop if bumper hit
		event = leftClif?ME_BUMP_LEFT:ME_BUMP_RIGHT; // return event that bumper is hit
	}

	return event; // return any events that were generated
}

// start a turnTo command
// make robot turn through an angle in radians (angle can be +ve or -ve)
// turn at ang_vel angular velocity (rad/s)(ang_vel shuold always be positive)
void turnTo(float angle, float ang_vel) {

	start_heading = heading;       // get starting heading
	target_heading = fabsf(angle); // get magnitude of target angle to turn through

	target_dist_2=0.0f; // target distance is 0 when turning
	driving=true;       // flag that we are making a turn

	if(angle<0.0f) {
		turn_ccw=false;  // set flag saying we are turning right
		drive(0.0f,-fabsf(ang_vel)); // desired turn is -ve angle so command robot to start turning to the right
	}
	else {
		turn_ccw=true; // set flag saying we are turning left
		drive(0.0f,fabsf(ang_vel));  //  desired turn is +ve angle so command robot to start turning to the left
	}
}


// start a driveTo command
// make the robot drive forward or backwards in a straight line at given distance( dist in m) at a given speed(lin_vel in m/s)
// to drive backwards make dist -ve, velocity should always be +ve
void driveTo(float dist, float lin_vel) {

	// get current pose as starting point
	start_pose_x = pose_x;
	start_pose_y = pose_y;

	lin_vel = fabsf(lin_vel); // make sure velocity is positive

	target_dist_2 = dist*dist; // use squared distance to save abs and sqrt when testing if move done
	target_heading= 0.0f;  // target turn is 0 when driving straight

	driving=true; // set flag to say we are driving

	if(dist < 0.0f) {
	   drive(-lin_vel,0.0f); // target distance is -ve so start driving backwards

	}
	else {
	   drive(lin_vel,0.0f); // target distance is +ve so start driving forwards
	}
}

// update the internal robot pose estimate
// use the inverse kinematics to calculate the new robot pose based on how farst each wheel is rotating
void updatePose(float DT) {

	float dl = enc_left.state.vel*DT*WHEEL_RADIUS; // compute left wheel distance moved from encoder angular velocity and update period
	float dr = enc_right.state.vel*DT*WHEEL_RADIUS;// compute right wheel distance moved from encoder angular velocity and update period

	float d = (dl+dr)/2.0f; // robot linear distance moved (m)

	float dt = (dr-dl)/WHEEL_BASE; // robot angle turned (rad)

    heading += dt; // update heading from angle turned

    // clamp heading between +-PI
	if(heading > M_PI_F) {
		heading -= M_2PI_F;
	}
	else if(heading <= -M_PI_F) {
		heading += M_2PI_F;
	}

	// compute new x,y pose from distance moved and new heading
    pose_x += d * cosf(heading);
	pose_y += d * sinf(heading);

}



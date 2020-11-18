/*
 * motors.h
 *
 *  Implements functions to control the motors and motor travel
 *  Implements forward and reverse kinematics to control robot speed, rotation and calculate a ego centric pose
 *
 *  Created on: Sep 28, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdbool.h>
#include <math.h>

// use float definition of PI
extern const float M_PI_F;

#define MAX_LIN_VEL 0.5f           //  maximum linear velocity m/s
#define MAX_ANG_VEL (2.0f*M_PI_F)  //  maximum angular velocity rad/s


// Events that are returned depending on conditions detected in the motor controller
typedef enum MotorEvents_t {
    ME_NONE=0,        // no event
	ME_STOP=1,        // stop command executed
	ME_DONE_TURN=2,   // a turnTo command has completed
	ME_DONE_DRIVE=4,  // a driveTo command has completed
	ME_BUMP_LEFT=8,   // left bumper detected a hit (motors will be stopped if sensor is enabled)
	ME_BUMP_RIGHT=16, // right bumper detected a hit (motors will be stopped if sensor is enabled)

	CE_M1=32,
	CE_M2=64,
	CE_M3=128

} MotorEvent;

void STOP(void); // stop the motors
void drive(float lin_vel, float ang_vel); // run the motors to achieve desired linear and angular robot velocities

void turnTo(float angle, float ang_vel); // make the robot turn a specified angle (rad) at a given angular velocity (rad/s)
void driveTo(float dist, float lin_vel); // drive the robot forward or backwards the given distance (m) at the given speed(m/s)

void setMotorSpeed(float left, float right); // set the individual speed of the left and right wheels (rad/s)

MotorEvent updateMotors(bool pid_update, float DT); // update the motor controller and if pid_update is true also update the PID controller

#endif /* INC_MOTORS_H_ */

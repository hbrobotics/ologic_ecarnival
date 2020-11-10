/*
 * motors.h
 *
 *  Created on: Sep 28, 2020
 *      Author: ralph
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include <stdbool.h>
#include <math.h>

#define MAX_LIN_VEL 0.5         //  m/s
#define MAX_ANG_VEL (2.0*M_PI)  //  rad/s


typedef enum MotorEvents_t {
    ME_NONE=0,
	ME_STOP=1,
	ME_DONE_TURN=2,
	ME_DONE_DRIVE=4,
	ME_BUMP_LEFT=8,
	ME_BUMP_RIGHT=16,

	CE_M1=32,
	CE_M2=64,
	CE_M3=128

} MotorEvent;

void STOP(void);
void drive(float lin_vel, float ang_vel);

void turnTo(float angle, float ang_vel);
void driveTo(float dist, float lin_vel);

void setMotorSpeed(float left, float right);

MotorEvent updateMotors(bool pid_update, float DT);

#endif /* INC_MOTORS_H_ */

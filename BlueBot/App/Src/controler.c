/*
 * controler.c
 *
 *  Created on: Oct 1, 2020
 *      Author: ralph
 */


#include "controler.h"
#include "edge_sensor.h"
#include "motors.h"

#define FWD_SPEED 0.1
#define BACK_SPEED 0.1
#define TURN_SPEED (0.6)

#define BACK_DIST (-0.04)
#define TURN_ANG (M_PI/2.0)

typedef enum State_t {
	ST_IDLE=0,

	ST_M1_FWD,
	ST_M1_BCK_L,
	ST_M1_BCK_R,
	ST_M1_TURN,


	ST_M2,
	ST_M3,

	ST_COMPLETE

} STATE;


STATE state = ST_IDLE;



void updateControler(MotorEvent event) {


	if(event & ME_STOP) {
		state = ST_IDLE;
	}

	switch(state) {

		case ST_IDLE:
			switch(event) {
				case CE_M1:
					drive(FWD_SPEED,0.0);
					state= ST_M1_FWD;
					break;

				case CE_M2:
					break;

				case CE_M3:
					break;

				default:
					break;
			}
			break;

		case ST_M1_FWD:
			switch(event) {
				case ME_BUMP_LEFT:
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);
					driveTo(BACK_DIST,BACK_SPEED);
					state= ST_M1_BCK_R;
					break;

				case ME_BUMP_RIGHT:
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);
					driveTo(BACK_DIST,BACK_SPEED);
					state= ST_M1_BCK_L;
					break;

				default:
					break;
			}
			break;

		case ST_M1_BCK_L:
			switch(event) {
				case ME_DONE_DRIVE:
					enableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);
					turnTo(TURN_ANG,TURN_SPEED);
					state= ST_M1_TURN;
					break;

				default:
					break;
			}
			break;

		case ST_M1_BCK_R:
			switch(event) {
				case ME_DONE_DRIVE:
					enableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);
					turnTo(-TURN_ANG,TURN_SPEED);
					state= ST_M1_TURN;
					break;

				default:
					break;
			}
			break;

		case ST_M1_TURN:
			switch(event) {
				case ME_DONE_TURN:
					drive(FWD_SPEED,0.0);
					state= ST_M1_FWD;
					break;

				case ME_BUMP_LEFT:
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);
					driveTo(BACK_DIST,BACK_SPEED);
					state= ST_M1_BCK_R;
					break;

				case ME_BUMP_RIGHT:
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);
					driveTo(BACK_DIST,BACK_SPEED);
					state= ST_M1_BCK_L;
					break;

				default:
					break;
			}
			break;


		case ST_M2:
			break;

		case ST_M3:
			break;

		case ST_COMPLETE:
			break;

		default:
			break;
	}
}

/*
 * controler.c
 *
 *  Implements the main robot state machine controller
 *
 *  Created on: Oct 1, 2020
 *      Author: Ralph Gnauck
 */


#include "controler.h"
#include "edge_sensor.h"
#include "motors.h"

// define the speeds used to implement robot behaviors
#define FWD_SPEED 0.1f
#define BACK_SPEED 0.1f
#define TURN_SPEED (0.6f)

extern const float M_PI_F; // use only float math version of PI

// distances/angles to use to make backup and turning moves
#define BACK_DIST (-0.04f)
#define TURN_ANG (M_PI_F/2.0f)

// define states of the state machine
typedef enum State_t {

	ST_IDLE=0, // stopped , waiting for a command


	// states to implement level 1 challenge
	ST_M1_FWD, // drive forward
	ST_M1_BCK_L, // backup and turn left
	ST_M1_BCK_R, // backup and turn right
	ST_M1_TURN,  // turning

	// states to implement level 2 challenge (not implemented)
	ST_M2,

	// states to implement level 3 challenge (not implemented)
	ST_M3,

	// states to implement victory dance at completion of challenge level (not implemented)
	ST_COMPLETE

} STATE;


static STATE state = ST_IDLE; // state variable of the FSM (start in idle)


// update the state machine
// input events to trigger state transitions are in events parameter
void updateControler(MotorEvent event) {


	if(event & ME_STOP) { // regardless of current state - STOP event Stops motors and forces FSM to IDLE state
		state = ST_IDLE;
	}

	switch(state) {

		case ST_IDLE: // idle - wait for event to start a challenge level
			switch(event) {
				case CE_M1:
					drive(FWD_SPEED,0.0f); // starting level 1 - just start driving forward
					state= ST_M1_FWD;
					break;

				case CE_M2: // level 2 (TBD)
					break;

				case CE_M3: // level 3 (TBD)
					break;

				default: // ignore other events
					break;
			}
			break;

		case ST_M1_FWD: // driving forward - stop if we reach the edge
			switch(event) {
				case ME_BUMP_LEFT: // left sensor detected edge
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT); // Disable sensors so we can backup
					driveTo(BACK_DIST,BACK_SPEED); // now backup
					state= ST_M1_BCK_R;            // next state will turn to right when backup is finished
					break;

				case ME_BUMP_RIGHT:  // right sensor detected edge
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT); // Disable sensors so we can backup
					driveTo(BACK_DIST,BACK_SPEED);  // now backup
					state= ST_M1_BCK_L; // next state will turn to left when backup is finished
					break;

				default: // ignore other events
					break;
			}
			break;

		case ST_M1_BCK_L: // currently backing up - will start to turn left when finished
			switch(event) {
				case ME_DONE_DRIVE: // backup finished
					enableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT); // re-enable edge sensors before we start turning
					turnTo(TURN_ANG,TURN_SPEED); // start a turn to the left
					state= ST_M1_TURN; // next state will wait till turn is finished
					break;

				default: // ignore other events
					break;
			}
			break;

		case ST_M1_BCK_R:  // currently backing up - will start to turn right when finished
			switch(event) {
				case ME_DONE_DRIVE: // backup finished
					enableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT);  // re-enable edge sensors before we start turning
					turnTo(-TURN_ANG,TURN_SPEED); // start a turn to the right
					state= ST_M1_TURN; // next state will wait till turn is finished
					break;

				default: // ignore other events
					break;
			}
			break;

		case ST_M1_TURN: // currently doing a turn, stop when turn completed or we hit an edge again
			switch(event) {
				case ME_DONE_TURN: // turn complete - just start driving forward again
					drive(FWD_SPEED,0.0f);
					state= ST_M1_FWD;
					break;

				case ME_BUMP_LEFT:  // left sensor detected edge
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT); // start backup and turn right sequence again
					driveTo(BACK_DIST,BACK_SPEED);
					state= ST_M1_BCK_R;
					break;

				case ME_BUMP_RIGHT: // right sensor detected edge
					disableEdgeSensors(BUMP_BIT_LEFT | BUMP_BIT_RIGHT); // start backup and turn left sequence again
					driveTo(BACK_DIST,BACK_SPEED);
					state= ST_M1_BCK_L;
					break;

				default: // ignore other events
					break;
			}
			break;


		case ST_M2: // (TBD)
			STOP();
			state= ST_IDLE;
			break;

		case ST_M3: // (TBD)
			STOP();
			state= ST_IDLE;
			break;

		case ST_COMPLETE: // (TBD)
			STOP();
			state= ST_IDLE;
			break;

		default: // somthings broken - STOP and reset FSM
			STOP();
			state= ST_IDLE;
			break;
	}
}

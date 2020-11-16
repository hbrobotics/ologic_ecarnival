/*
 * ui.h
 *
 *  User interface handling module
 *
 *  Receives and processes input commands and transmits robot status to a client
 *
 *  Created on: Oct 15, 2020
 *      Author: Ralph Gnauck
 */

#include "motors.h"
#include "encoder.h"
#include "pid.h"


void doUI(uint8_t * packet, int len, MotorEvent *event); // Process an input packet and respond to commands
void sendTelemetry(void); // send robot telemetry to client

void setEncoderState(ENCODER_STATE * enc_left, ENCODER_STATE * enc_right); // save current encoder state info
void setPIDState(PID_STATE * pid_left, PID_STATE * pid_right);// save current PID state info
void setMotorState(void);// save current motor state info
void setControlerState(void);// save current controller state info
void setIRRangeState(float range_long, float range_short); // save current ir sensor state info

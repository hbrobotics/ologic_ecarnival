/*
 * ui.h
 *
 *  Created on: Oct 15, 2020
 *      Author: ralph
 */

#include "motors.h"
#include "encoder.h"
#include "pid.h"

void doUI(uint8_t * packet, int len, MotorEvent *event);
void sendTelemetry(void);

void setEncoderState(ENCODER_STATE * enc_left, ENCODER_STATE * enc_right);
void setPIDState(PID_STATE * pid_left, PID_STATE * pid_right);
void setMotorState();
void setControlerState();
void setIRRangeState(float range_long, float range_short);

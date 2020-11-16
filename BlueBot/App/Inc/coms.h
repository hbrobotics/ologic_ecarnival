/*
 * coms.h
 *
 * process coms on the UART
 *
 *  Data sent/received over the UART is packetized and passed in SLIP encoded format
 *
 *  Incoming packets are passed to the UI module to be decoded
 *
 *  The telemetry status of the robot is sent as an output packet
 *
 *
 *   NOTE: Slip encoding used includes unique START and END characters on each packet
 *
 *  Created on: Oct 15, 2020
 *      Author: Ralph Gnauck
 */

#ifndef INC_COMS_H_
#define INC_COMS_H_

#include <stdio.h>
#include <stdbool.h>
#include "motors.h"


MotorEvent doComs(void); // handle the coms (called from main loop)

// helpers to encode/decode data packets in slip encoded format
void slipEncode(uint8_t * buf, int len);
bool slipDecode(uint8_t c, int size, uint8_t * slipInPacket, int * out_len);

#endif /* INC_COMS_H_ */

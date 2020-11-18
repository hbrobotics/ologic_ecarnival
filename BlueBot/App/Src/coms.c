/*
 * coms.c
 *
 *
 * process coms on the UART
 *
 *  Data sent/received over the UART is packetized and passed in SLIP encoded format
 *
 *  Incoming packets are passed to the UI module to be decoded
 *
 *  The telemetry status of the robot is sent as an output packet
 *
 *  NOTE: Slip encoding used includes unique START and END characters on each packet
 *
 *  Created on: Oct 15, 2020
 *      Author: Ralph Gnauck
 */

#include "coms.h"
#include "pid.h"
#include"encoder.h"
#include "motors.h"
#include "usart.h"
#include "ui.h"

// declare special characters used by protocol
#define SLIP_END 0xC0
#define SLIP_START 0xC1
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_START 0xDE
#define SLIP_ESC_ESC 0xDD

#define COMS_UART huart1 // map the UART to use for the COMS stream

// Define STATE Variable values for SLIP decoding state machine
typedef enum SLIP_RX_STATE_t {
    SRX_IDLE=0,
    SRX_ESC,
    SRX_CHAR
} SLIP_RX_STATE;

// mintain the state of the decoded packet
static SLIP_RX_STATE slip_rx_state=SRX_IDLE; // parser state machine state
static int rx_idx=0; // input buffer offset to store next decoded character

// Buffers for the module to use to encode and decode SLIP packets
#define PACKET_SIZE 128
static uint8_t coms_in_buffer[PACKET_SIZE];
static uint8_t coms_out_buffer[PACKET_SIZE];

// local prototypes
static int read_uart(void);


// called from main loop to process incoming data from the COMS UART
// when a full input packet is received it is passed to the UI module to be processed
// If the UI generates an event it is returned from this function
MotorEvent doComs(void) {

	MotorEvent event=0;
	int c = read_uart(); // get char from UART (if any)
	if(c != -1) { // check if valid

		int in_len; // gets set to length of decoded input packet by slipDecode if a complete packet is received
		if(slipDecode((uint8_t)c,PACKET_SIZE,coms_in_buffer,&in_len)) { // add decoded data to the input buffer
			doUI(coms_in_buffer,in_len,&event); // got end of packet so pass it to UI module
		}
	}

	return event; // return events raised by UI to main app loop for processing

}


// decode the data passed into the function
// characters (c) from a slip encoded stream should be passed to this function one at a time
// the function maintains an internal state of the encoded packet.
// When a valid packet is fully parsed it will return true.
// the decoded characters are stored in the buffer slipInPacket supplied by the caller
// size should be set to the max length of the buffer pointed to by slipInPacket
// when a complete input packet is parsed out_len is updated to the length of the decoded packet and the function returns true
//
// NOTE - only one packet can be decoded at once as the global variables rx_idx and slip_rx_state are used to maintain the state of the packet being decoded.
//
bool slipDecode(uint8_t c, int size, uint8_t * slipInPacket, int * out_len) {

   *out_len=0;

   if(rx_idx >= size) { // make sure we don't over run the supplied buffer, ignore current packet and reset for new packet if we do
       slip_rx_state=SRX_IDLE;
       rx_idx=0;
   }

   switch(slip_rx_state) {

       case SRX_IDLE: // wait till we see a START char to begin decodeing data
           rx_idx = 0;
           if ( c == SLIP_START) {
               slip_rx_state = SRX_CHAR;
           }
           break;

       case SRX_ESC: // if we just got an ESC, decode the character that was sent

           if (c == SLIP_ESC_ESC) { // decode escaped ESC
              slipInPacket[rx_idx++] = SLIP_ESC;
              slip_rx_state = SRX_CHAR;
           }

           else if (c == SLIP_ESC_END) {  // decode escaped END
               slipInPacket[rx_idx++]  = SLIP_END;
               slip_rx_state = SRX_CHAR;
           }

           else if (c == SLIP_ESC_START) {   // decode escaped START
               slipInPacket[rx_idx++]  = SLIP_START;
               slip_rx_state= SRX_CHAR;
           }

           else {
              slip_rx_state = SRX_IDLE; // unexpected character - ignore packet and wait for next one
           }

           break;

       case SRX_CHAR: // got character, check if it is a special character

           if (c == SLIP_END) { // found an end char so return true and set out_len
               *out_len=rx_idx; // return size of packet to caller
               rx_idx= 0;  // reset for next packet
               slip_rx_state = SRX_IDLE;
               return true; // return true to say we got a full packet
           }
           else if (c == SLIP_ESC) { // its an ESC so goto the ESC state to decode next character
               slip_rx_state = SRX_ESC;
           }
           else if (c == SLIP_START) { // got unexpected start, ignore packet and wiat till next
               slip_rx_state = SRX_IDLE;
           }
           else {
               slipInPacket[rx_idx++]  = c; // just a normal char- save in the decoded buffer
           }

           break;

   }

   return false; // not yet full packet so return false
}


// Helper macro to put character in output buffer
#define SLIP_SEND(c)	coms_out_buffer[tx_idx++] = c

// encodes and transmits a packet of data in slip format.
// buf points to the buffer with the raw data
// len - length in bytes of the data buffer
// encoded packet is sent out over the COMS uart
void slipEncode(uint8_t * buf, int len)  {
int tx_idx=0;

    SLIP_SEND(SLIP_START); // Add Slip start character

    for(int idx=0; idx < len; idx++) {

        int c = *buf++;

        if (c== SLIP_END) { // encode an escaped END character
            SLIP_SEND(SLIP_ESC);
            SLIP_SEND(SLIP_ESC_END);
        }
        else if (c== SLIP_ESC) { // encode an escaped ESC character
            SLIP_SEND(SLIP_ESC);
            SLIP_SEND(SLIP_ESC_ESC);
        }
        else if (c== SLIP_START) { // encode an escaped START character
            SLIP_SEND(SLIP_ESC);
            SLIP_SEND(SLIP_ESC_START);
        }
        else {
            SLIP_SEND(c); // just output normal character
        }
    }

    SLIP_SEND(SLIP_END); // ADD Slip END to terminate the packet


    HAL_UART_Transmit_DMA(&COMS_UART,coms_out_buffer,tx_idx); // transmit the encoded packet using DMA
}


// Read a character from the COMS Uart
// returns the character if it can be read
// returns -1 if no characters have been received
int read_uart(void) {

	if(__HAL_UART_GET_FLAG(&COMS_UART, UART_FLAG_RXNE)) { // see if UART has a character
		uint8_t ch=0;

		__HAL_UART_CLEAR_FEFLAG(&COMS_UART); // clear status and error flags (ignoring errors !)
		__HAL_UART_CLEAR_OREFLAG(&COMS_UART);
		__HAL_UART_CLEAR_PEFLAG(&COMS_UART);

		HAL_StatusTypeDef sts = HAL_UART_Receive(&COMS_UART,&ch,1,1); // read the character
		if(sts == HAL_OK) {
			return (int)ch; // if ok return the character
		}
	}
	return -1; // nothing to read or error
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  // should do something here
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // don't need to know if packet has finished being sent
}

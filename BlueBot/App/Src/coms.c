/*
 * coms.c
 *
 *  Created on: Oct 15, 2020
 *      Author: ralph
 */
#include "coms.h"
#include "pid.h"
#include"encoder.h"
#include "motors.h"
#include "usart.h"
#include "ui.h"

#define SLIP_END 0xC0
#define SLIP_START 0xC1
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_START 0xDE
#define SLIP_ESC_ESC 0xDD

#define COMS_UART huart1

typedef enum SLIP_RX_STATE_t {
    SRX_IDLE=0,
    SRX_ESC,
    SRX_CHAR
} SLIP_RX_STATE;

SLIP_RX_STATE slip_rx_state=SRX_IDLE;
int rx_idx=0;

#define PACKET_SIZE 128
uint8_t coms_in_buffer[PACKET_SIZE];
uint8_t coms_out_buffer[PACKET_SIZE];

static int read_uart(void);

MotorEvent doComs(void) {

	MotorEvent event=0;
	int c = read_uart();
	if(c != -1) {
		int in_len;
		if(slipDecode((uint8_t)c,PACKET_SIZE,coms_in_buffer,&in_len)) {
			doUI(coms_in_buffer,in_len,&event);
		}
	}

	return event;

}


// decode the data passed into the function
// characters (c) from a slip encoded stream should be passed to this function one at a time
// the function maintains an internal state of the encoded packet.
// When a valid packet is fully parsed it will return true.
// the decoded packet is saved in the global buffer rxBuffer[dev] associated with the file stream indicated by sci_fd parameter
//

bool slipDecode(uint8_t c, int size, uint8_t * slipInPacket, int * out_len) {

   *out_len=0;

   if(rx_idx >= size) {
       slip_rx_state=SRX_IDLE;
       rx_idx=0;
   }

   switch(slip_rx_state) {

       case SRX_IDLE:
           rx_idx = 0;
           if ( c == SLIP_START) {
               slip_rx_state = SRX_CHAR;
           }
           break;

       case SRX_ESC:

           if (c == SLIP_ESC_ESC) {
              slipInPacket[rx_idx++] = SLIP_ESC;
              slip_rx_state = SRX_CHAR;
           }

           else if (c == SLIP_ESC_END) {
               slipInPacket[rx_idx++]  = SLIP_END;
               slip_rx_state = SRX_CHAR;
           }

           else if (c == SLIP_ESC_START) {
               slipInPacket[rx_idx++]  = SLIP_START;
               slip_rx_state= SRX_CHAR;
           }

           else {
              slip_rx_state = SRX_IDLE; // unexpected character - ignore packet and wait for next one
           }

           break;

       case SRX_CHAR:

           if (c == SLIP_END) {
               *out_len=rx_idx;
               rx_idx= 0;
               slip_rx_state = SRX_IDLE;
               return true;
           }
           else if (c == SLIP_ESC) {
               slip_rx_state = SRX_ESC;
           }
           else if (c == SLIP_START) {
               slip_rx_state = SRX_IDLE;
           }
           else {
               slipInPacket[rx_idx++]  = c;
           }

           break;

   }

   return false;
}


// encodes and transmits a packet of data in slip format.
// buf points to the buffer with the raw data
// len - length in bytes of the data buffer
// sci_fd - file pointer to transmit the data on


#define SLIP_SEND(c)	coms_out_buffer[tx_idx++] = c


void slipEncode(uint8_t * buf, int len)  {
int tx_idx=0;

    SLIP_SEND(SLIP_START);

    for(int idx=0; idx < len; idx++) {

        int c = *buf++;

        if (c== SLIP_END) {
            SLIP_SEND(SLIP_ESC);
            SLIP_SEND(SLIP_ESC_END);
        }
        else if (c== SLIP_ESC) {
            SLIP_SEND(SLIP_ESC);
            SLIP_SEND(SLIP_ESC_ESC);
        }
        else if (c== SLIP_START) {
            SLIP_SEND(SLIP_ESC);
            SLIP_SEND(SLIP_ESC_START);
        }
        else {
            SLIP_SEND(c);
        }
    }

    SLIP_SEND(SLIP_END);


    HAL_UART_Transmit_DMA(&COMS_UART,coms_out_buffer,tx_idx);
}



int read_uart(void) {

	if(__HAL_UART_GET_FLAG(&COMS_UART, UART_FLAG_RXNE)) {
		uint8_t ch=0;

		__HAL_UART_CLEAR_FEFLAG(&COMS_UART);
		__HAL_UART_CLEAR_OREFLAG(&COMS_UART);
		__HAL_UART_CLEAR_PEFLAG(&COMS_UART);

		HAL_StatusTypeDef sts = HAL_UART_Receive(&COMS_UART,&ch,1,1);
		if(sts == HAL_OK) {
			return (int)ch;
		}
	}
	return -1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}

#ifndef _IRreceive_H_
#define _IRreceive_H_
#include "config.h"
//#include Num 2

#define __DEBUG        0
#define RAWBUF         300 // Length of raw duration buffer
#define MARK_EXCESS    100
#define STATE_IDLE     2
#define STATE_MARK     3
#define STATE_SPACE    4
#define STATE_STOP     5

#define MARK  0
#define SPACE 1

// len, start_H, start_L, nshort, nlong, data_len, data[data_len]....
#define D_LEN       0
#define D_STARTH    1
#define D_STARTL    2
#define D_SHORT     3
#define D_LONG      4
#define D_DATALEN   5
#define D_DATA      6

typedef struct decode_results_parameter_all
{
	volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
	int rawlen;           // Number of records in rawbuf.
}decode_results_parameter;

typedef struct IRSendRev_parameter_all
{
	decode_results_parameter	results;
	Gpio_pin_parameter			recvpin;         // pin for IR data from detector
	u8							rcvstate;        // state machine
	unsigned int				timer;           // state timer, counts 50uS ticks.
	unsigned int				rawbuf[RAWBUF];  // raw data
	u8							rawlen;          // counter of entries in rawbuf
}IRSendRev_parameter;

//**************************rev**********************************
void IRSendRev_myinterrupt( IRSendRev_parameter* IRSendRev_para );
void IRSendRev_Init( IRSendRev_parameter* IRSendRev_para, Gpio_pin_parameter* revPin);                
unsigned char IRSendRev_Recv( IRSendRev_parameter* IRSendRev_para, unsigned char *revData);     // 
unsigned char IRSendRev_IsDta( IRSendRev_parameter* IRSendRev_para );                          // if IR get data
void IRSendRev_Clear( IRSendRev_parameter* IRSendRev_para );                                   // clear IR data

#endif

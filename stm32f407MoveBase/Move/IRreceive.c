#include "IRreceive.h"
#include "config.h"
int IRSendRev_decode( IRSendRev_parameter* IRSendRev_para, decode_results_parameter *results );
void IRSendRev_enableIRIn( IRSendRev_parameter* IRSendRev_para );  

void IRSendRev_myinterrupt( IRSendRev_parameter* IRSendRev_para )
{
  u8 irdata = (u8)GPIO_ReadInputDataBit(IRSendRev_para->recvpin.m_gpio, IRSendRev_para->recvpin.m_pin);

  IRSendRev_para->timer++; // One more 50us tick
  if (IRSendRev_para->rawlen >= RAWBUF) {
    // Buffer overflow
    IRSendRev_para->rcvstate = STATE_STOP;
  }
  switch(IRSendRev_para->rcvstate) {
	  case STATE_IDLE: // In the middle of a gap
		if (irdata == MARK) {
			if (IRSendRev_para->timer < 100) {
			// Not big enough to be a gap.
			IRSendRev_para->timer = 0;
			} 
			else {
			// gap just ended, record duration and start recording transmission
			IRSendRev_para->rawlen = 0;
			IRSendRev_para->rawbuf[IRSendRev_para->rawlen++] = IRSendRev_para->timer;
			IRSendRev_para->timer = 0;
			IRSendRev_para->rcvstate = STATE_MARK;
			}
		}
		break;
  case STATE_MARK: // timing MARK
    if (irdata == SPACE) {   // MARK ended, record time
      IRSendRev_para->rawbuf[IRSendRev_para->rawlen++] = IRSendRev_para->timer;
      IRSendRev_para->timer = 0;
      IRSendRev_para->rcvstate = STATE_SPACE;
    }
    break;
  case STATE_SPACE: // timing SPACE
    if (irdata == MARK) { // SPACE just ended, record it
      IRSendRev_para->rawbuf[IRSendRev_para->rawlen++] = IRSendRev_para->timer;
      IRSendRev_para->timer = 0;
      IRSendRev_para->rcvstate = STATE_MARK;
    }
    else { // SPACE
      if (IRSendRev_para->timer > 100) {
        // big SPACE, indicates gap between codes
        // Mark current code as ready for processing
        // Switch to STOP
        // Don't reset timer; keep counting space width
        IRSendRev_para->rcvstate = STATE_STOP;
      } 
    }
    break;
  case STATE_STOP: // waiting, measuring gap
    if (irdata == MARK) { // reset gap timer
      IRSendRev_para->timer = 0;
    }
    break;
  }
  //Timer1.start();
}

int IRSendRev_decode( IRSendRev_parameter* IRSendRev_para, decode_results_parameter *results) {
  results->rawbuf = IRSendRev_para->rawbuf;
  results->rawlen = IRSendRev_para->rawlen;
  if (IRSendRev_para->rcvstate != STATE_STOP) {
    return 0;
  }
  // Throw away and start over
  IRSendRev_Clear(IRSendRev_para);
  return 1;
}

void IRSendRev_Init( IRSendRev_parameter* IRSendRev_para, Gpio_pin_parameter* revPin)
{
    IRSendRev_para->recvpin.m_gpio    = revPin->m_gpio;
	IRSendRev_para->recvpin.m_pin    = revPin->m_pin;
    
    IRSendRev_enableIRIn(IRSendRev_para); // Start the receiver
    delay(20);
    IRSendRev_Clear(IRSendRev_para);
}

unsigned char IRSendRev_IsDta( IRSendRev_parameter* IRSendRev_para )
{

    if(IRSendRev_decode(IRSendRev_para, &IRSendRev_para->results))
    {
        int count  = IRSendRev_para->results.rawlen;
        if(count < 64 || (count -4)%8 != 0)
        {
#if __DEBUG
            Serial.print("IR GET BAD DATA!\r\n");
#endif
            IRSendRev_Clear(IRSendRev_para);        // Receive the next value
            return 0;
        }
        int count_data  = (count-4) / 16;
#if __DEBUG
        Serial.print("ir get data! count_data = ");
        Serial.println(count_data);
#endif
        return (unsigned char)(count_data+6);
    }
    else 
    {
        return 0;
    }

}

void IRSendRev_Clear( IRSendRev_parameter* IRSendRev_para ) {
  IRSendRev_para->rcvstate = STATE_IDLE;
  IRSendRev_para->rawlen = 0;
}

void IRSendRev_enableIRIn( IRSendRev_parameter* IRSendRev_para ) {
  //cli();
  // setup pulse clock timer interrupt
  //Prescale /8 (16M/8 = 0.5 microseconds per tick)
  // Therefore, the timer interval can range from 0.5 to 128 microseconds
  // depending on the reset value (255 to 0)
 // ----TIMER_CONFIG_NORMAL();

  //Timer2 Overflow Interrupt Enable
 // ---TIMER_ENABLE_INTR;

 // ---TIMER_RESET;
 //修改部分
  
  //sei();  // enable interrupts

  // initialize state machine variables
  IRSendRev_para->rcvstate = STATE_IDLE;
  IRSendRev_para->rawlen = 0;

  // set pin modes
  pinInputModeInit(&IRSendRev_para->recvpin);
}

unsigned char IRSendRev_Recv( IRSendRev_parameter* IRSendRev_para, unsigned char *revData)
{
    int count       = IRSendRev_para->results.rawlen;
    int nshort      = 0;
    int nlong       = 0;
    int count_data  = 0;

    count_data = (count-4)/16;

    for(int i = 0; i<10; i++)           // count nshort
    {
        nshort += IRSendRev_para->results.rawbuf[3+2*i];
    }
    nshort /= 10;

    int i = 0;
    int j = 0;
    while(1)        // count nlong
    {
        if(IRSendRev_para->results.rawbuf[4+2*i] > (2*nshort))
        {
            nlong += IRSendRev_para->results.rawbuf[4+2*i];
            j++;
        }
        i++;
        if(j==10)break;
        if((4+2*i)>(count-10))break;
    }
    nlong /= j;

    int doubleshort = 2*nshort;
    for(i = 0; i<count_data; i++)
    {
        revData[i+D_DATA] = 0x00;
        for(j = 0; j<8; j++)
        {
            if(IRSendRev_para->results.rawbuf[4 + 16*i + j*2] > doubleshort) // 1
            {
                revData[i+D_DATA] |= 0x01<< (7-j);
            }
            else
            {
                revData[i+D_DATA] &= ~(0x01<<(7-j));
            }
        }
    }
    revData[D_LEN]      = count_data+5;
    revData[D_STARTH]   = IRSendRev_para->results.rawbuf[1];
    revData[D_STARTL]   = IRSendRev_para->results.rawbuf[2];
    revData[D_SHORT]    = nshort;
    revData[D_LONG]     = nlong;
    revData[D_DATALEN]  = count_data;
 
#if __DEBUG
    Serial.print("\r\n*************************************************************\r\n");
    Serial.print("len\t = ");Serial.println(revData[D_LEN]);
    Serial.print("start_h\t = ");Serial.println(revData[D_STARTH]);
    Serial.print("start_l\t = ");Serial.println(revData[D_STARTL]);
    Serial.print("short\t = ");Serial.println(revData[D_SHORT]);
    Serial.print("long\t = ");Serial.println(revData[D_LONG]);
    Serial.print("data_len = ");Serial.println(revData[D_DATALEN]);
    for(int i = 0; i<revData[D_DATALEN]; i++)
    {
        Serial.print(revData[D_DATA+i]);Serial.print("\t");
    }
    Serial.print("\r\n*************************************************************\r\n");
#endif

    IRSendRev_Clear(IRSendRev_para); // Receive the next value
    return revData[D_LEN]+1;
}


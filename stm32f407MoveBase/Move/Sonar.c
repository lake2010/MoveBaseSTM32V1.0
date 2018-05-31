#include "Sonar.h"
#include "I2C.h"
#include "MoveBase.h"
#include "AutomaticCharge.h"

const int m_CenterDistance[6] = {0,0,0,0,0,0};
const int m_SonarAngle[6] = {1,2,3,0,0,0};

void CSonar_echoCheck( CSonar_parameter* CSonar_para );
int CSonar_outputResult( CSonar_parameter* CSonar_para );
void CSonar_SendRead( CSonar_parameter* CSonar_para, int address );
void CSonar_ReadDistance( CSonar_parameter* CSonar_para, int address, unsigned int *Distance );
void CSonar_SonarOneSetup( CSonar_parameter* CSonar_para, int address );
void CSonar_CaTemp( CSonar_parameter* CSonar_para, byte high, byte low );


void CSonar_setup( CSonar_parameter* CSonar_para )
{
	CSonar_para->m_bReady = false;
	CSonar_para->m_event = 0;
	CSonar_para->m_fEventProcessed = false;
	CSonar_para->m_warning = 0;
	CSonar_para->m_minDistance = 200;
	CSonar_para->m_wait_time = 0;
	CSonar_para->m_vcontrol = 0;
	CSonar_para->m_lastvcontrol = 0;

	CSonar_para->m_currentSensor = 0;
	CSonar_para->m_pingTimer[CSonar_para->m_currentSensor] = millis() + 100; // First ping starts at 100ms, gives time for the Arduino to chill before starting.
	CSonar_para->m_wait_time = CSonar_para->m_detTimer = millis();
	CSonar_para->m_start_temp = millis();
	CSonar_para->m_start_tmp = true;
}

void CSonar_SonarOneSetup( CSonar_parameter* CSonar_para, int address )
{
	address = address >> 1;
	I2C_writeByte(I2C1, address, m_REG, m_INT);
}

void CSonar_loop( CSonar_parameter* CSonar_para)
{
	char*    buf = 0;
	int      len;
	if( CMySerial_getCmd_reeman(&MoveBase.m_canccm.Can_ccm, &buf, &len) ){
		unsigned char crc = 0;
		for( int i = 2; i < (len-1); i++ ){
			crc ^=buf[i];
		}	
		if( crc != buf[len-1] )
			return;
		switch(buf[3])
		{
			case CMD_OP_SONAR_DATA:		
				mySerialWriteUSART4((byte *)buf,len);	
				myprintfUSART1("%d,%d,%d\r\n",((uint16_t)buf[4]<<8)+buf[5],((uint16_t)buf[6]<<8)+buf[7],((uint16_t)buf[8]<<8)+buf[9]);
				break;
		}
	}
}

void CSonar_ReadDistance( CSonar_parameter* CSonar_para, int address, unsigned int *Distance )
{
	byte highByte, lowByte, highlow[2];
	
	I2C_readBytes( I2C1, CSonar_para->m_SONAR_KS103_ADDRESS[CSonar_para->m_currentSensor] >> 1, m_REG, 2, highlow); 
	highByte = highlow[0];// Get high byte
	lowByte = highlow[1];// Get low byte
	*Distance = ((highByte << 8) + lowByte);

	*Distance = (int)( (double)*Distance/(double)20000*((double)25*0.6 + 331.3) );
}

int CSonar_getEvent( CSonar_parameter* CSonar_para )
{
	return CSonar_para->m_event;
}

int CSonar_getWarning( CSonar_parameter* CSonar_para, unsigned int *m_minDis )
{
	int waring;
	waring = CSonar_para->m_warning;
	CSonar_para->m_warning = 0;
	*m_minDis = CSonar_para->m_minDistance;
	return waring;
}

int CSonar_outputResult( CSonar_parameter* CSonar_para )
{
	// if no object is detected, output nothing.
	CSonar_para->m_minDistance = 200;
	int i;
	for (i = 0; i < m_SONAR_NUM; i++) {
		if (CSonar_para->m_cm[i] > 0)
			break;
	}
	if (i == m_SONAR_NUM)
		return 0;

	for( int j = 0; j < m_SONAR_NUM; j++ ){
		if( CSonar_para->m_cm[j] != 0 )
			LIMIT_MAX( CSonar_para->m_minDistance, CSonar_para->m_cm[j] );
	}

	int ret = 0;
	myprintf( "@sonar" );
	int CenterDistance = 0;
	for (i = 0; i < m_SONAR_NUM; i++) {
		if( CSonar_para->m_cm[i] == 0 ){
			CenterDistance = 0;
		}else{
			CenterDistance = m_CenterDistance[i];
		}
		myprintf( " %d %d", m_SonarAngle[i], (CSonar_para->m_cm[i] +  CenterDistance) );
		if (0 < CSonar_para->m_cm[i] && CSonar_para->m_cm[i] <= m_WARNING_DIST)
		  ret += 1<<i;
    }
	myprintf( "#\r\n" );
	return ret;
}

void CSonar_CaTemp( CSonar_parameter* CSonar_para, byte high, byte low )
{
	byte mhigh = high & 0x80;
	//负温度
	unsigned int mtemp = (high << 8) | low;
	if( mhigh ){
		mtemp = ~mtemp;
		mtemp++;
		CSonar_para->m_sonar_temp = -(double)mtemp * 0.0625;
	}else{
		CSonar_para->m_sonar_temp = mtemp * 0.0625;
	}
	sonarprintf( ("m_sonar_temp:%d\n", (int)CSonar_para->m_sonar_temp) );
}

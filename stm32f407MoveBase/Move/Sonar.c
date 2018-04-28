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
	I2C1_Init();

	CSonar_para->m_bReady = false;
	CSonar_para->m_event = 0;
	CSonar_para->m_fEventProcessed = false;
	CSonar_para->m_warning = 0;
	CSonar_para->m_minDistance = 200;
	CSonar_para->m_wait_time = 0;
	CSonar_para->m_vcontrol = 0;
	CSonar_para->m_lastvcontrol = 0;

	CSonar_para->m_SONAR_KS103_ADDRESS[0] = 0xd0;
	CSonar_para->m_SONAR_KS103_ADDRESS[1] = 0xd2;
	CSonar_para->m_SONAR_KS103_ADDRESS[2] = 0xd4;

	CSonar_para->m_currentSensor = 0;

	CSonar_SonarOneSetup(CSonar_para, CSonar_para->m_SONAR_KS103_ADDRESS[0]);
	CSonar_SonarOneSetup(CSonar_para, CSonar_para->m_SONAR_KS103_ADDRESS[1]);
	CSonar_SonarOneSetup(CSonar_para, CSonar_para->m_SONAR_KS103_ADDRESS[2]);
	I2C_writeByte( I2C1, CSonar_para->m_SONAR_KS103_ADDRESS[0] >> 1, 2, 0xc3 );
	I2C_writeByte( I2C1, CSonar_para->m_SONAR_KS103_ADDRESS[1] >> 1, 2, 0xc3 );
	I2C_writeByte( I2C1, CSonar_para->m_SONAR_KS103_ADDRESS[2] >> 1, 2, 0xc3 );
	
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

void CSonar_loop( CSonar_parameter* CSonar_para, int nCmdOp, const int* pParam, int nParamNum )
{
	//咱先注释CODO2PG_loop( &MoveBase.m_odo, nCmdOp, pParam, nParamNum );
	/*--读取指令屏蔽超声波控速--*/
	if( nCmdOp == CMD_OP_SONARCONTROL ){
		if( nParamNum == 1 ){
			if( pParam[0] == 1 ){
				CSonar_para->m_vcontrol = 1;
				CSonar_para->m_lastvcontrol = 1;
			}else if( pParam[0] == 0 ){
				CSonar_para->m_vcontrol = 0;
				CSonar_para->m_lastvcontrol = 0;
			}else
				myprintf("sonarcontrol data num must be 0 or 1\r\n");
			sonarprintf(( "m_vcontrol:%d", CSonar_para->m_vcontrol ));
		}else{
			myprintf("sonarcontrol data num must be One..0 or 1\r\n");
		}
	}
	/*--充电时关闭超声波，充电结束打开超声波并开控速，向前走以后控速状态还原--*/
	boolean ac_isChargeing = CAutoCharge_isChargeing(&MoveBase.m_ac);
	int ac_isGotoWork = CAutoCharge_isGotoWork(&MoveBase.m_ac);
	if( ac_isGotoWork )
		CSonar_para->m_vcontrol = 1;
	else
		CSonar_para->m_vcontrol = CSonar_para->m_lastvcontrol;

	if( ac_isChargeing && !ac_isGotoWork )
		return;

	if (! CSonar_para->m_bReady) {
		/*--10s进行一次测温--*/
		if( millis() - CSonar_para->m_start_temp >= 30000 ){
			CSonar_para->m_start_tmp = true;
			CSonar_para->m_det_temp = millis();
			CSonar_para->m_start_temp = millis();
			CSonar_para->m_currentSensor = 0;
		}
		if( CSonar_para->m_start_tmp ){
			if( millis() - CSonar_para->m_det_temp > 10 ){
				CSonar_para->m_det_temp = millis();
				I2C_writeByte( I2C1, CSonar_para->m_SONAR_KS103_ADDRESS[CSonar_para->m_currentSensor] >> 1, m_REG, m_SONAR_TEMP );
				CSonar_para->m_currentSensor++;
				if (CSonar_para->m_currentSensor >= m_SONAR_NUM) {
					CSonar_para->m_bReady = true;
					CSonar_para->m_currentSensor = 0;
				}
			}
			return;
		}
		/*--没有测温则进行距离测量--*/
		I2C_writeByte( I2C1, CSonar_para->m_SONAR_KS103_ADDRESS[CSonar_para->m_currentSensor] >> 1, m_REG, m_SONAR_MODE );
		CSonar_para->m_wait_time = millis();
		CSonar_para->m_bReady = true;
	} else {
		if( CSonar_para->m_start_tmp ){
			if( millis() - CSonar_para->m_det_temp > 100 ){
				byte highlow[2];
				I2C_readBytes( I2C1, CSonar_para->m_SONAR_KS103_ADDRESS[CSonar_para->m_currentSensor] >> 1, m_REG, 2, highlow);
				byte high = highlow[0];//0x02是高位
				byte low = highlow[1];//0x03是低位
				CSonar_CaTemp( CSonar_para, high, low );
				CSonar_para->m_currentSensor++;
				if( CSonar_para->m_currentSensor >= m_SONAR_NUM ){
					CSonar_para->m_start_tmp = false;
					CSonar_para->m_bReady = false;
					CSonar_para->m_currentSensor = 0;
				}
			}
			return;
		}

		if( millis() - CSonar_para->m_wait_time >= 40 ){// Request 2 bytes from SRF module
			//if( I2c.available() >= 1 ){
				CSonar_ReadDistance( CSonar_para, CSonar_para->m_SONAR_KS103_ADDRESS[CSonar_para->m_currentSensor], &CSonar_para->m_cm[CSonar_para->m_currentSensor] );
				if (0 < CSonar_para->m_cm[CSonar_para->m_currentSensor] && CSonar_para->m_cm[CSonar_para->m_currentSensor] <= m_WARNING_DIST) {
					CSonar_para->m_event = 1 << CSonar_para->m_currentSensor; // set warning event
					CSonar_para->m_minDistance = CSonar_para->m_cm[CSonar_para->m_currentSensor];
				}
				if (0 < CSonar_para->m_cm[CSonar_para->m_currentSensor] && CSonar_para->m_cm[CSonar_para->m_currentSensor] <= m_STOP_DIST) {
					CSonar_para->m_warning = 1; // set warning
				}
			/*}else{
				CSonar_para->m_cm[CSonar_para->m_currentSensor] = 0;
			}*/
			CSonar_para->m_currentSensor++;
			CSonar_para->m_bReady = false;
			if (CSonar_para->m_currentSensor >= m_SONAR_NUM) {
				CSonar_para->m_event = CSonar_outputResult( CSonar_para );
				CSonar_para->m_currentSensor = 0;
			}
		}
	}
	
	if( !CSonar_para->m_vcontrol ){
		CSonar_para->m_event = 0;
		CSonar_para->m_warning = 0;
		return;
	}

	if (CSonar_para->m_event > 0) {
		sonarprintf(( "in sonar event\n" ));
		CSonar_para->m_fEventProcessed = true;
		int vx, vy, wz;
		CMove2_getv( &MoveBase.m_move, &vx, &vy, &wz );
		if( vx > 600 )
			vx = 600;
		int pParam[] = {0, 0, 0};
		if ( 0 < vx && m_STOP_DIST < CSonar_para->m_minDistance && CSonar_para->m_minDistance <= m_WARNING_DIST ) {
			sonarprintf(( "setv low\n" ));
			pParam[0] = vx / 10;
			if (  0 < pParam[0] && pParam[0] <= 25 ){
				pParam[0] = 0;
				CMove2_stop( &MoveBase.m_move );
				delay(300);
				CMove2_start( &MoveBase.m_move );
			}
			CMove2_setv(&MoveBase.m_move, pParam[0], pParam[1], pParam[2]);
			CODO2PG_loop( &MoveBase.m_odo);
		}else if( 0 < vx && 0 < CSonar_para->m_minDistance && CSonar_para->m_minDistance <= m_STOP_DIST ){
			sonarprintf(( "Sonar stop\n" ));
			CMove2_stop( &MoveBase.m_move );
			delay(300);
			CMove2_start( &MoveBase.m_move );
		}
	}
	//m_minDistance = 200;
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

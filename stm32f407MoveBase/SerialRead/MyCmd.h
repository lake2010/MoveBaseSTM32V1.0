#ifndef _MYCMD_H
#define _MYCMD_H

//cmd信息定义
#ifdef __cplusplus  
extern "C" {  
#endif

#define MAX_CMD_STRLEN		32
#define MAX_CMD_PARAM_NUM	8

#define CMD_STR_START			("start")
#define CMD_STR_START2			("start2")
#define CMD_STR_SET_V			("setv")
#define CMD_STR_SET_S			("sets")
#define CMD_STR_STOP			("stop")
#define CMD_STR_MOTOR_DISABLE ("motordis")
#define CMD_STR_DEC				("dec")
#define CMD_STR_GOTOX			("gotox")
#define CMD_STR_GOTOY			("gotoy")
#define CMD_STR_ROTATE			("rotate")
#define CMD_STR_COFF_ON			("coffon")
#define CMD_STR_COFF_OFF		("coffoff")
#define CMD_STR_AC_ON			("acon")
#define CMD_STR_AC_OFF			("acoff")
#define CMD_STR_WORK			("gotowork")
#define CMD_STR_SONARCONTROL	("sonarcontrol")
#define CMD_STR_ELEVATORUP		("eleup")
#define CMD_STR_ELEVATORDOWN	("eledown")
#define CMD_STR_ELEVATORTEST	("eletest")
#define CMD_STR_ELEVATORSTOP	("elestop")

typedef enum {
//#endif
	CMD_OP_START = 101,
	CMD_OP_START2,
	CMD_OP_SET_V,
	CMD_OP_SET_S,
	CMD_OP_STOP,
	CMD_OP_SET_V2,
 	CMD_OP_MOTOR_DISABLE,
	CMD_OP_AC_ON,
	CMD_OP_AC_OFF,
	/*   BooBase上发给SLAM信息  */
	CMD_OP_ODO2PG,
	CMD_OP_OBD,
	CMD_OP_SONAR_DATA,
	CMD_OP_SYS_STATUS,
	CMD_OP_FAULT_CODE,
	
	CMD_OP_DEC,
	CMD_OP_GOTOX,
	CMD_OP_GOTOY,
	CMD_OP_ROTATE,
	CMD_OP_COFF_ON,
	CMD_OP_COFF_OFF,
	CMD_OP_WORK,
	CMD_OP_SONARCONTROL,
	CMD_OP_NUM,
	CMD_OP_ELEVATORUP,
	CMD_OP_ELEVATORDOWN,
	CMD_OP_ELEVATORTEST,
	CMD_OP_ELEVATORSTOP,
} CMD_OP;

typedef struct _CMD_STRING {
	char str[MAX_CMD_STRLEN];
} CMD_STRING;

#ifdef __cplusplus  
}  
#endif

#endif 




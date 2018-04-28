#ifndef _SerialCom_H
#define _SerialCom_H

#include "SerialBuf.h"
#include "ICmd.h"

#ifdef __cplusplus  
extern "C" {  
#endif  

#define m_MAX_CMD_NUM  5

typedef struct CSerialCom_param_all
{	
    CMD_STRING m_pCmd[m_MAX_CMD_NUM];
    int m_nCmdNum;//init zero
    CSerialBuf_param m_SerialBuf;
}CSerialCom_param;

void CSerialCom_recv( CSerialCom_param* CSerialCom_para );
CMD_STRING* CSerialCom_getCmd( CSerialCom_param* CSerialCom_para, int *nCmdNum );

#ifdef __cplusplus  
}  
#endif

#endif //_SerialCom_H


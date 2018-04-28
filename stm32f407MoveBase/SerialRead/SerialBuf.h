#ifndef _SerialBuf_H
#define _SerialBuf_H

#ifdef __cplusplus  
extern "C" {  
#endif  

#define m_BUF_LEN  30

typedef struct CSerialBuf_param_all
{
    char m_pBuf[m_BUF_LEN];
    int m_nDataLen;//init zero
}CSerialBuf_param;

void CSerialBuf_AddByte( CSerialBuf_param* CSerialBuf_par, char x );
int  CSerialBuf_GetPackage( CSerialBuf_param* CSerialBuf_par, char* pBuf, int nBufLen );

#ifdef __cplusplus  
}  
#endif

#endif //_SerialBuf_H


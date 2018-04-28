#include "SerialBuf.h"
#include <string.h>

void CSerialBuf_AddByte( CSerialBuf_param* CSerialBuf_par, char x )
{
  if (CSerialBuf_par->m_nDataLen >= m_BUF_LEN) {
    memcpy( CSerialBuf_par->m_pBuf, CSerialBuf_par->m_pBuf+1, CSerialBuf_par->m_nDataLen-1 );
    CSerialBuf_par->m_nDataLen--;
  }
  CSerialBuf_par->m_pBuf[CSerialBuf_par->m_nDataLen++] = x;
}

int CSerialBuf_GetPackage( CSerialBuf_param* CSerialBuf_par, char* pBuf, int nBufLen )
{
	char* p;
	int nDataLen;

	p = CSerialBuf_par->m_pBuf;
	while (p != CSerialBuf_par->m_pBuf + CSerialBuf_par->m_nDataLen && *p != '@') {
		p++;
	}
	if (p == CSerialBuf_par->m_pBuf + CSerialBuf_par->m_nDataLen) {
		CSerialBuf_par->m_nDataLen = 0;
		return 0;
	}
	memcpy( CSerialBuf_par->m_pBuf, p, CSerialBuf_par->m_nDataLen - (p - CSerialBuf_par->m_pBuf) );
	CSerialBuf_par->m_nDataLen -= (p - CSerialBuf_par->m_pBuf);

	p = CSerialBuf_par->m_pBuf+1;
	while (p != CSerialBuf_par->m_pBuf + CSerialBuf_par->m_nDataLen && *p != '#') {
		p++;
	}
	if (p == CSerialBuf_par->m_pBuf + CSerialBuf_par->m_nDataLen)
		return 0;

	nDataLen = p - CSerialBuf_par->m_pBuf + 1;
	memcpy( pBuf, CSerialBuf_par->m_pBuf, nBufLen );
	memcpy( CSerialBuf_par->m_pBuf, CSerialBuf_par->m_pBuf + nDataLen, CSerialBuf_par->m_nDataLen - nDataLen );
	CSerialBuf_par->m_nDataLen -= nDataLen;

	if (nDataLen > nBufLen) // @@
	  return 0;

	return nDataLen;
}


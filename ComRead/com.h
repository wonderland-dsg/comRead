#include "windows.h"
#include <stdio.h>
#include <iostream>
//openmp֧��
#include <omp.h>
/********�������װ********/

class PortOperate
{
public:
	PortOperate()
	{
	}
	~PortOperate()
	{}
	//�ܳ�ʼ������
	bool Initial(char* port);
	/***************
	���ڹ��ܺ���:
	��opencomm
	��ȡReadData
	д��WriteData
	�ر�CloseComm
	***************/
	int OpenComm(char* port);

	int ReadData(unsigned char *pBuffer, unsigned char ulen);

	int WriteData(unsigned char *pBuffer, unsigned char uLen);

	void CloseComm();
public:
	HANDLE A_hCom;
	unsigned char str[100];
	unsigned char tmpchar[12];
};
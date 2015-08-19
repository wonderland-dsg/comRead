#include "windows.h"
#include <stdio.h>
#include <iostream>
//openmp支持
#include <omp.h>
/********操作类封装********/

class PortOperate
{
public:
	PortOperate()
	{
	}
	~PortOperate()
	{}
	//总初始化函数
	bool Initial(char* port);
	/***************
	串口功能函数:
	打开opencomm
	读取ReadData
	写入WriteData
	关闭CloseComm
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
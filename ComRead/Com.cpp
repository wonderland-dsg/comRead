
#include "com.h"


int PortOperate::OpenComm(char* port)
{
	A_hCom = CreateFile(port,
		GENERIC_READ | GENERIC_WRITE, //�������д
		0, //��ռ��ʽ
		NULL,
		OPEN_EXISTING, //�򿪶����Ǵ���
		0, //ͬ����ʽ
		NULL);
	if (A_hCom == INVALID_HANDLE_VALUE)
	{
		DWORD dwErr = GetLastError();
		printf("opencom failed %d\n", dwErr);
		return FALSE;
	}
	else
	{
		// �򿪳ɹ�����ʼ������
		DCB wdcb = { 0 };
		GetCommState(A_hCom, &wdcb); //��ȡ��ǰ���ڲ���
		wdcb.BaudRate = CBR_9600;         // ������
		wdcb.ByteSize = 8;                  // ����λ8
		wdcb.fBinary = TRUE;				// �����Ʒ�ʽ
		wdcb.fParity = FALSE;
		wdcb.Parity = NOPARITY;			// ����żУ��
		wdcb.StopBits = ONESTOPBIT;        //1ֹͣλ
		//	wdcb.fRtsControl = false;
		//	wdcb.fDtrControl = false;
		//	wdcb.fOutxCtsFlow = false;
		//	wdcb.fOutxDsrFlow = false;
		wdcb.XonLim = 2048;
		wdcb.XoffLim = 512;
		wdcb.EofChar = 0;
		// ���ô��ڲ���
		SetCommState(A_hCom, &wdcb);

		// ���ô��ڳ�ʱ����
		COMMTIMEOUTS to =                   // ���ڳ�ʱ���Ʋ���
		{
			0,                       // ���ַ������ʱʱ��
			100,                              // ������ʱÿ�ַ���ʱ��
			0,                              // �����ģ�����ģ�����ʱʱ��
			MAXDWORD,                       // д����ʱÿ�ַ���ʱ��
			10                               // �����ģ�����ģ�д��ʱʱ��
		};
		SetCommTimeouts(A_hCom, &to);
		// ���ô��ڻ������
		SetupComm(A_hCom, 1024, 1024);
		// ��ղ��������ڵ�ǰ����
		PurgeComm(A_hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);
		printf("open com done\n");
		//m_hCom = hCom;						// ������
	}
	return 0;
}
int PortOperate::ReadData(unsigned char* pBuffer, unsigned char ulen)
{
	// �Ӵ��ڶ�ȡ����
	DWORD dwRead;
	memset(str, 0x00, sizeof(str));
	if (!ReadFile(A_hCom, pBuffer, ulen, &dwRead, NULL))
	{
		DWORD dwErr = GetLastError();
		//printf("%ld\n",dwErr);
		printf("read error %ld\n", dwErr);
		return dwErr;
	}
	//printf("read done %d\n",dwRead);
	//	PurgeComm(A_hCom, PURGE_TXCLEAR|PURGE_RXCLEAR);
	return 0;
}
int PortOperate::WriteData(unsigned char *pBuffer, unsigned char uLen)
{
	// д�����ݵ�����
	DWORD dwWritten;
	if (uLen > 0)
	{
		dwWritten = 0;
		if (!WriteFile(A_hCom, pBuffer, uLen, &dwWritten, NULL))
		{
			DWORD dwErr = GetLastError();
			printf("wirte error %ld\n", dwErr);
			return dwErr;
		}
		else{
			printf("write done %d\n", dwWritten);
		}
	}
	PurgeComm(A_hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);
	return 0;
}
void PortOperate::CloseComm()
{
	CloseHandle(A_hCom);
	A_hCom = NULL;
}
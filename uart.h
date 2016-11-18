#ifndef __UART_H 
#define __UART_H

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04
#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

extern volatile unsigned char timeUART3WaitPacket;

struct TALARMS {
	unsigned char Low_P_O2;			//	0 ��� - ������� �������� �2�
	unsigned char Hi_P_O2;			//	1 ��� - �������� �������� �2�
	unsigned char Low_P_He;			//	2 ��� - ������� �������� He�
	unsigned char Hi_P_He;			//	3 ��� - �������� �������� He�
	unsigned char Low_P_P_O2;		//	6 ��� - ������� ������������ %�2�
	unsigned char Hi_P_P_O2;		//	7 ��� - �������� ���������������� %�2�
	unsigned char Ingalation_off;	// ��������� �����������
	unsigned char Dangerous_pressure_in_the_mask;	//4 ��� ������� 2 ���c��� �������� � �����
	unsigned char APNOE; 			//5 ��� ������� 2 �����
	unsigned char HighLeak; 		//6 ��� ������� 2 ������ ������ �2
	unsigned char Datch_ULTRASONIC_error; //7 ��� ������������� ��������������� �������
	unsigned char Datch_O2_neispr;	//	0 ��� - ��������� ������ %�2�
	unsigned char Datch_O2_calibr;	//	1 ��� - ����� ���������� ������� %�2�
	unsigned char Device_fail;		//	2 ��� - �������� ����������������
};

struct TRXDATA {
	unsigned char PvxO2;
	unsigned char PvxHe;
	short Pm;
	unsigned char O2f;
	unsigned char Tf;
	unsigned short V;
	unsigned char F;
	unsigned short V_skor;
	unsigned char HeaterTemp;
	unsigned short O2flow;
	unsigned short Heflow;
	unsigned short O2concEtalon;
	unsigned short O2concFast;
	unsigned short ADC1_array[8];
	unsigned short ADC2_array[8];
	struct TALARMS Alarms;
	unsigned short alarmsFromBU;
};
typedef struct TRXDATA TRxData;
extern volatile TRxData RxData;

extern volatile unsigned char sendToPColdProtocol;

unsigned long UART0Init(unsigned long Baudrate);
void UART0Handler(void) __attribute__ ((interrupt ("IRQ")));
void sendDataToPCold();

extern volatile unsigned char UART3dataIsReady;
unsigned long UART3Init(unsigned long Baudrate);
void UART3Handler(void) __attribute__ ((interrupt ("IRQ")));
void readDataFromBU();
void sendDataToBU();


#endif

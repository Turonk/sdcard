/*****************************************************************************
 *   uart.h:  Header file for NXP LPC23xx Family Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2006.09.01  ver 1.00    Prelimnary version, first Release
 *
 ******************************************************************************/
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

#define BUFSIZE		0x40

struct TALARMS {
	unsigned char Low_P_O2;			//	0 бит - УЌизкое давление ќ2Ф
	unsigned char Hi_P_O2;			//	1 бит - У¬ысокое давление ќ2Ф
	unsigned char Low_P_He;			//	2 бит - УЌизкое давление HeФ
	unsigned char Hi_P_He;			//	3 бит - У¬ысокое давление HeФ
	unsigned char Low_P_Xe;			//	4 бит - УЌизкое давление XeФ
	unsigned char Hi_P_Xe;			//	5 бит - У¬ысокое давление XeФ
	unsigned char Low_P_P_O2;		//	6 бит - УЌизка€ концентраци€ %ќ2Ф
	unsigned char Hi_P_P_O2;		//	7 бит - У¬ысока€ концконцентраци€ %ќ2Ф
	unsigned char Ingalation_off;	// »нгал€ци€ остановлена
	unsigned char Dangerous_pressure_in_the_mask;	//4 бит сигналы 2 ќпаcное давление в маске
	unsigned char APNOE; 			//5 бит сигналы 2 јѕЌќЁ
	unsigned char Datch_Old; 		//6 бит сигналы 2 —тарый датчик ќ2
	unsigned char Datch_ULTRASONIC_error; //7 бит неисправность ультрозвукового датчика
};
typedef struct TALARMS TAlarms;

struct TINFO {
	unsigned char Datch_O2_neispr;	//	0 бит - У«амените датчик %ќ2Ф
	unsigned char Datch_O2_calibr;	//	1 бит - У»дет калибровка датчика %ќ2Ф
	unsigned char Device_fail;		//	2 бит - Ујппарат неработоспособенФ
};
typedef struct TINFO TInfo;

struct TRXDATATIME {
	unsigned long RTC_Sec; 			/* Second value - [0,59] */
	unsigned long RTC_Min; 			/* Minute value - [0,59] */
	unsigned long RTC_Hour; 		/* Hour value - [0,23] */
	unsigned long RTC_Mday; 		/* Day of the month value - [1,31] */
	unsigned long RTC_Mon; 			/* Month value - [1,12] */
	unsigned long RTC_Year; 		/* Year value - [0,4095] */
	unsigned long RTC_Wday; 		/* Day of week value - [0,6] */
	unsigned long RTC_Yday; 		/* Day of year value - [1,365] */
	unsigned char new;
};
typedef struct TRXDATATIME TRXDataTime;

struct TRXDATA {
	unsigned char PvxO2;
	unsigned char PvxHe;
	unsigned char Pm;
	unsigned char O2f;
	unsigned char Tf;
	unsigned short V;
	unsigned char F;
	TAlarms Alarms;
	TInfo Info;
	unsigned short V_skor;
	unsigned char rezerv1;
	unsigned short trevogi_summ;
	unsigned char byte_alarms;
	unsigned char byte_info;

	unsigned char HeaterTemp;
	unsigned short O2flow;
	unsigned short Heflow;
	unsigned short O2concEtalon;
	unsigned short O2concFast;
	unsigned short ADC1_array[8];
	unsigned short ADC2_array[8];

	TRXDataTime RxDataTime;
};

typedef struct __attribute__((packed)) {
	char ind1;
	char ind2;
	unsigned short V;
	unsigned char O2f;
	unsigned char Tf;
	unsigned char Ff;
}TTX_Data_komp;
typedef struct TRXDATA TRxData;
extern TRxData RxData;

typedef struct __attribute__((packed)) {
	TTX_Data_komp Buf_UART;
	unsigned short count;
}TBuf_uart0;
TBuf_uart0 Buf_uart0;

unsigned long UARTInit(unsigned long portNum, unsigned long Baudrate);
void UART0Handler(void) __attribute__ ((interrupt ("IRQ")));
void UART1Handler(void);
void UART3Handler(void) __attribute__ ((interrupt ("IRQ")));
void UARTSend(unsigned long portNum, unsigned char *BufferPtr, unsigned long Length);
char UART0_tx_pc(TTX_Data_komp TX_Data);
BYTE getByte_info_2();
void sendDataToPCold();
void dataHandling();

#endif

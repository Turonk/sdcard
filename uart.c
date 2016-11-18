#include "LPC24xx.h"
#include "type.h"
#include "target.h"
#include "irq.h"
#include "uart.h"
#include "rtc.h"
#include "spi.h"
#include "board.h"
#include "diskio.h"
#include "ff.h"
#include "screens.h"
#include "blackbox.h"

volatile TRxData RxData;		//данные прин€тые от блока управлени€
volatile DWORD UART0Status;
volatile unsigned char UART0TxEmpty = 1;
volatile unsigned char UART0RxBuffer[70];
volatile unsigned char UART0RxCounter = 0;
volatile unsigned char UART0packetState = 0;
volatile unsigned char UART0TxBuffer[70];
volatile unsigned char UART0TxCount = 0;
volatile unsigned char UART0TxCounter = 0;
unsigned char versionBU;
unsigned char flags1;
#define IN_PACKET_SIZE 67
#define OUT_PACKET_SIZE 61
volatile DWORD UART3Status;
volatile unsigned char UART3TxEmpty = 1;
volatile unsigned char UART3RxBuffer[IN_PACKET_SIZE];
volatile unsigned char UART3readyData[IN_PACKET_SIZE];
volatile unsigned char UART3dataIsReady = 0;
volatile unsigned char UART3RxCounter = 0;
volatile unsigned char UART3TxBuffer[OUT_PACKET_SIZE];
volatile unsigned char UART3TxCount = 0;
volatile unsigned char UART3TxCounter = 0;
volatile unsigned char timeUART3WaitPacket = 0;	//врем€ от предыдущего макета от системы управлени€

extern unsigned char stAlarm;
extern unsigned char rej_alarm;
extern unsigned int CL_BACKGROUND;

volatile unsigned char sendToPColdProtocol;

// »нициализаци€ UART 0
unsigned long UART0Init(unsigned long baudrate) {
	PINSEL0 |= 0x00000050; 	// RxD0 and TxD0
	U0LCR = 0x83; 			// 8 bits, no Parity, 1 Stop bit
	//DWORD Fdiv = (Fpclk / 16) / baudrate; //*baud rate *///15000000//16
	U0DLM = 0;				//Fdiv / 256;
	U0DLL = 6;				//Fdiv % 256;
	U0FDR = 7 + (12 << 4);	//DIVADDVAL + (MULVAL<<4);//
	U0LCR = 0x03; 			//DLAB = 0
	U0FCR = 0x07; 			//Enable and reset TX and RX FIFO.

	if (install_irq(UART0_INT, (void *) UART0Handler, HIGHEST_PRIORITY) == FALSE) {
		return (FALSE);
	}
	U0IER = IER_RBR | IER_THRE | IER_RLS; //Enable UART0 interrupt
	return (TRUE);
}

// отправка данных на компьютер по протоколу старой программы мониторинга
void sendDataToPCold(){
	UART0TxBuffer[0] = 0xAA;
	UART0TxBuffer[1] = 0xBB;
	UART0TxBuffer[2] = RxData.V & 0xFF;
	UART0TxBuffer[3] = RxData.V >> 8;
	UART0TxBuffer[4] = RxData.O2concEtalon/10;
	UART0TxBuffer[5] = RxData.Tf;
	UART0TxBuffer[6] = RxData.F;
	UART0TxCount = 7;
	UART0TxCounter = 0;
	U0THR = UART0TxBuffer[UART0TxCounter];
}

// ќбработчик прерывани€ от UART0 (св€зь с компьютером)
void UART0Handler(void) {
	BYTE IIRValue, LSRValue;
	BYTE Dummy = Dummy;
	BYTE dataReceive = 0;

	//IENABLE;				/* handles nested interrupt */
	IIRValue = U0IIR;
	IIRValue >>= 1; 		/* skip pending bit in IIR */
	IIRValue &= 0x07; 		/* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS){ /* Receive Line Status */
		LSRValue = U0LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART0Status = LSRValue;
			Dummy = U0RBR; /* Dummy read on RX to clear interrupt, then bail out */
			// IDISABLE;
			VICVectAddr = 0; /* Acknowledge Interrupt */
			return;
		}
		if (LSRValue & LSR_RDR){ /* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			dataReceive = 1;
		}
	}else if (IIRValue == IIR_RDA){ /* Receive Data Available */
		dataReceive = 1;
	}else if (IIRValue == IIR_CTI){ /* Character timeout indicator */
		UART0Status |= 0x100; 	/* Bit 9 as the CTI error */
	}else if (IIRValue == IIR_THRE){ /* THRE, transmit holding register empty */
		LSRValue = U0LSR; /* Check status in the LSR to see if valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UART0TxEmpty = 1;

			UART0TxCounter++;
			if (UART0TxCounter<UART0TxCount) U0THR = UART0TxBuffer[UART0TxCounter];
		}else {
			UART0TxEmpty = 0;
		}
	}

	if (dataReceive){
		BYTE rxbyte = U0RBR;
		switch (UART0packetState) {
			case 0:
				if (rxbyte == 0xAA) UART0packetState++;
				else UART0packetState = 0;
				break;
			case 1:
				if (rxbyte == 0xBB) UART0packetState++;
				else UART0packetState = 0;
				UART0RxCounter = 0;
				break;
			case 2:
				UART0RxBuffer[UART0RxCounter] = rxbyte;
				UART0RxCounter++;
				if (UART0RxCounter >= UART0RxBuffer[0]){
					if (UART0RxCounter>2){
						BYTE i = 0;
						BYTE contrsumm = 0;
						for (i = 0; i < UART0RxCounter - 1; i++) contrsumm += UART0RxBuffer[i];
						if (contrsumm == UART0RxBuffer[UART0RxCounter - 1]){
							BYTE rxCommand = UART0RxBuffer[1];
							if (rxCommand==1 || rxCommand==7 || rxCommand==11){

								BYTE packetSize;
								if (rxCommand == 1){
									packetSize = 56;
									UART0TxBuffer[3] = 1;
								}else if (rxCommand == 7){
									packetSize = 24;
									UART0TxBuffer[3] = 7;
								}else {
									packetSize = 66;
									UART0TxBuffer[3] = 11;
								}

								UART0TxBuffer[0] = 0xAA;
								UART0TxBuffer[1] = 0xBB;
								UART0TxBuffer[2] = packetSize;
								UART0TxBuffer[4] = RxData.PvxO2;
								UART0TxBuffer[5] = RxData.PvxHe;
								UART0TxBuffer[6] = RxData.O2flow & 0xFF;
								UART0TxBuffer[7] = RxData.O2flow >> 8;
								UART0TxBuffer[8] = RxData.Heflow & 0xFF;
								UART0TxBuffer[9] = RxData.Heflow >> 8;
								UART0TxBuffer[10] = RxData.O2concEtalon & 0xFF;
								UART0TxBuffer[11] = RxData.O2concEtalon >> 8;
								UART0TxBuffer[12] = RxData.O2concFast & 0xFF;
								UART0TxBuffer[13] = RxData.O2concFast >> 8;
								UART0TxBuffer[14] = RxData.Pm;
								UART0TxBuffer[15] = RxData.Tf;
								UART0TxBuffer[16] = RxData.HeaterTemp;
								UART0TxBuffer[17] = RxData.V & 0xFF;
								UART0TxBuffer[18] = RxData.V >> 8;
								UART0TxBuffer[19] = RxData.F;
								UART0TxBuffer[20] = data[0];
								UART0TxBuffer[21] = data[1];
								UART0TxBuffer[22] = RxData.alarmsFromBU;
								UART0TxBuffer[23] = RxData.alarmsFromBU>>8;
								UART0TxBuffer[24] = (Dev_st==dev_work) | ((data[1]>0)<<1) | ((st_KEY2==0)<<2) | ((timeUART3WaitPacket>=5)<<3);

								unsigned char arrconter = 25;

								if (rxCommand == 1 || rxCommand == 11){
									unsigned char i;

									for(i = 0; i < 8; i++ ){
										UART0TxBuffer[arrconter] = RxData.ADC1_array[i] & 0xFF;
										arrconter++;
										UART0TxBuffer[arrconter] = RxData.ADC1_array[i] >> 8;
										arrconter++;
									}

									for(i = 0; i < 8; i++ ){
										UART0TxBuffer[arrconter] = RxData.ADC2_array[i] & 0xFF;
										arrconter++;
										UART0TxBuffer[arrconter] = RxData.ADC2_array[i] >> 8;
										arrconter++;
									}
								}

								if (rxCommand == 11){
									UART0TxBuffer[arrconter] = capnoGetControlByte();
									arrconter++;
									UART0TxBuffer[arrconter] = capnoState;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoConfig;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoErrors;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoCO2/10;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoFiCO2;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoEtCO2;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoFiCO2pr;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoEtCO2pr;
									arrconter++;
									UART0TxBuffer[arrconter] = capnoRR;
									arrconter++;
								}

								contrsumm = 0;
								for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
								UART0TxBuffer[packetSize + 1] = contrsumm;
								UART0TxCount = packetSize + 2;
								UART0TxCounter = 0;
								U0THR = UART0TxBuffer[UART0TxCounter];
							}else if (rxCommand == 2){	//запрос текущего времени
								BYTE packetSize = 10;
								UART0TxBuffer[0] = 0xAA;
								UART0TxBuffer[1] = 0xBB;
								UART0TxBuffer[2] = packetSize;
								UART0TxBuffer[3] = 0x02;
								RTCTime Real_timer = RTCGetTime();
								UART0TxBuffer[4] = Real_timer.RTC_Hour;
								UART0TxBuffer[5] = Real_timer.RTC_Min;
								UART0TxBuffer[6] = Real_timer.RTC_Sec;
								UART0TxBuffer[7] = Real_timer.RTC_Mday;
								UART0TxBuffer[8] = Real_timer.RTC_Mon;
								UART0TxBuffer[9] = Real_timer.RTC_Year & 0xFF;
								UART0TxBuffer[10] = Real_timer.RTC_Year >>8;
								contrsumm = 0;
								for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
								UART0TxBuffer[packetSize + 1] = contrsumm;
								UART0TxCount = packetSize + 2;
								UART0TxCounter = 0;
								U0THR = UART0TxBuffer[UART0TxCounter];
							}else if (rxCommand == 3){ //установка текущего времени
								RTCTime Real_timer = RTCGetTime();
								Real_timer.RTC_Hour = UART0RxBuffer[2];
								Real_timer.RTC_Min = UART0RxBuffer[3];
								Real_timer.RTC_Sec = UART0RxBuffer[4];
								Real_timer.RTC_Mday = UART0RxBuffer[5];
								Real_timer.RTC_Mon = UART0RxBuffer[6];
								Real_timer.RTC_Year = (UART0RxBuffer[8]<<8) + UART0RxBuffer[7];
								Real_timer.RTC_Wday = UART0RxBuffer[9];
								Real_timer.RTC_Yday = (UART0RxBuffer[11]<<8) + UART0RxBuffer[10];
								RTCSetTime(Real_timer);

								BYTE packetSize = 3;
								UART0TxBuffer[0] = 0xAA;
								UART0TxBuffer[1] = 0xBB;
								UART0TxBuffer[2] = packetSize;
								UART0TxBuffer[3] = 0x03;
								contrsumm = 0;
								for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
								UART0TxBuffer[packetSize + 1] = contrsumm;
								UART0TxCount = packetSize + 2;
								UART0TxCounter = 0;
								U0THR = UART0TxBuffer[UART0TxCounter];
							}else if (rxCommand == 4){ //чтение поправок
								BYTE packetSize = 39;
								UART0TxBuffer[0] = 0xAA;
								UART0TxBuffer[1] = 0xBB;
								UART0TxBuffer[2] = packetSize;
								UART0TxBuffer[3] = 4;

								unsigned char j=4;
								for (i = 0; i<9; i++){
									UART0TxBuffer[j] = poprArray[0][i] & 0xFF;
									j++;
									UART0TxBuffer[j] = poprArray[0][i] >> 8;
									j++;
								}

								for (i = 0; i<9; i++){
									UART0TxBuffer[j] = poprArray[1][i] & 0xFF;
									j++;
									UART0TxBuffer[j] = poprArray[1][i] >> 8;
									j++;
								}

								contrsumm = 0;
								for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
								UART0TxBuffer[packetSize + 1] = contrsumm;
								UART0TxCount = packetSize + 2;
								UART0TxCounter = 0;
								U0THR = UART0TxBuffer[UART0TxCounter];
							}else if (rxCommand == 5){
								poprArray[0][0] = (UART0RxBuffer[3]<<8) + UART0RxBuffer[2];
								poprArray[0][1] = (UART0RxBuffer[5]<<8) + UART0RxBuffer[4];
								poprArray[0][2] = (UART0RxBuffer[7]<<8) + UART0RxBuffer[6];
								poprArray[0][3] = (UART0RxBuffer[9]<<8) + UART0RxBuffer[8];
								poprArray[0][4] = (UART0RxBuffer[11]<<8) + UART0RxBuffer[10];
								poprArray[0][5] = (UART0RxBuffer[13]<<8) + UART0RxBuffer[12];
								poprArray[0][6] = (UART0RxBuffer[15]<<8) + UART0RxBuffer[14];
								poprArray[0][7] = (UART0RxBuffer[17]<<8) + UART0RxBuffer[16];
								poprArray[0][8] = (UART0RxBuffer[19]<<8) + UART0RxBuffer[18];
								poprArray[1][0] = (UART0RxBuffer[21]<<8) + UART0RxBuffer[20];
								poprArray[1][1] = (UART0RxBuffer[23]<<8) + UART0RxBuffer[22];
								poprArray[1][2] = (UART0RxBuffer[25]<<8) + UART0RxBuffer[24];
								poprArray[1][3] = (UART0RxBuffer[27]<<8) + UART0RxBuffer[26];
								poprArray[1][4] = (UART0RxBuffer[29]<<8) + UART0RxBuffer[28];
								poprArray[1][5] = (UART0RxBuffer[31]<<8) + UART0RxBuffer[30];
								poprArray[1][6] = (UART0RxBuffer[33]<<8) + UART0RxBuffer[32];
								poprArray[1][7] = (UART0RxBuffer[35]<<8) + UART0RxBuffer[34];
								poprArray[1][8] = (UART0RxBuffer[37]<<8) + UART0RxBuffer[36];

								writePopr();

								BYTE packetSize = 3;
								UART0TxBuffer[0] = 0xAA;
								UART0TxBuffer[1] = 0xBB;
								UART0TxBuffer[2] = packetSize;
								UART0TxBuffer[3] = 5;
								contrsumm = 0;
								for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
								UART0TxBuffer[packetSize + 1] = contrsumm;
								UART0TxCount = packetSize + 2;
								UART0TxCounter = 0;
								U0THR = UART0TxBuffer[UART0TxCounter];
							}else if (rxCommand == 6){
								BYTE packetSize = 15;
								UART0TxBuffer[0] = 0xAA;
								UART0TxBuffer[1] = 0xBB;
								UART0TxBuffer[2] = packetSize;
								UART0TxBuffer[3] = 6;
								UART0TxBuffer[4] = (st_SD_STAT>0) | (disk_mount_result == FR_OK)<<1 | (file_open_result == FR_OK)<<2 ;
								UART0TxBuffer[5] = 0;
								UART0TxBuffer[6] = tot_sect;
								UART0TxBuffer[7] = tot_sect>>8;
								UART0TxBuffer[8] = tot_sect>>16;
								UART0TxBuffer[9] = tot_sect>>24;
								UART0TxBuffer[10] = fre_sect;
								UART0TxBuffer[11] = fre_sect>>8;
								UART0TxBuffer[12] = fre_sect>>16;
								UART0TxBuffer[13] = fre_sect>>24;
								UART0TxBuffer[14] = (disk_file_count?disk_file_count-1:0);
								UART0TxBuffer[15] = (disk_file_count?disk_file_count-1:0)>>8;
								contrsumm = 0;
								for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
								UART0TxBuffer[packetSize + 1] = contrsumm;
								UART0TxCount = packetSize + 2;
								UART0TxCounter = 0;
								U0THR = UART0TxBuffer[UART0TxCounter];
							}else if (rxCommand == 8){
								sendToPColdProtocol = 1;
							}else if (rxCommand == 9){
								sendToPColdProtocol = 0;
							}else if (rxCommand == 10){

							}else if(rxCommand ==14){
								CL_BACKGROUND =(UART0RxBuffer[3]<<8) + UART0RxBuffer[2];
								CL_TEXT = (UART0RxBuffer[5]<<8) + UART0RxBuffer[4];
								CL_MENU = (UART0RxBuffer[7]<<8) + UART0RxBuffer[6];
								CL_BUTTON = (UART0RxBuffer[9]<<8) + UART0RxBuffer[8];
								CL_ALLOCATE = (UART0RxBuffer[11]<<8) + UART0RxBuffer[10];
								CL_INDICATION = (UART0RxBuffer[13]<<8) + UART0RxBuffer[12];
								CL_GRAPH = (UART0RxBuffer[15]<<8) + UART0RxBuffer[14];
								CL_GRAPH_OSI= (UART0RxBuffer[17]<<8) + UART0RxBuffer[16];
								CL_GRAPH_OSI_TEXT = (UART0RxBuffer[19]<<8) + UART0RxBuffer[18];
								CL_GRAPH_DOT_LINE = (UART0RxBuffer[21]<<8) + UART0RxBuffer[20];
								CL_CALIBR = (UART0RxBuffer[23]<<8) + UART0RxBuffer[22];
								CL_STOP = (UART0RxBuffer[25]<<8) + UART0RxBuffer[24];
								CL_SELECTION = (UART0RxBuffer[27]<<8) + UART0RxBuffer[26];
								goToScreen(Window);
							}


						}
					}
					UART0packetState = 0;
				}
				break;
		}
	}

	// IDISABLE;
	VICVectAddr = 0; /* Acknowledge Interrupt */
}

// »нициализаци€ UART 3
unsigned long UART3Init(unsigned long baudrate) {
	PCONP |= (1 << 25);		//¬ключить питание дл€ usart3
	PINSEL0 |= 0x0000000A; 	//RxD0 and TxD0 */
	U3LCR = 0x83; 			//8 bits, no Parity, 1 Stop bit */
	//DWORD Fdiv = (Fpclk / 16) / baudrate; /*baud rate *///15000000//16
	U3DLM = 0;				//Fdiv / 256;
	U3DLL = 6;				//Fdiv % 256;
	U3FDR = 7 + (12 << 4);	//DIVADDVAL + (MULVAL<<4);
	U3LCR = 0x03; 			//DLAB = 0
	U3FCR = 0x07; 			//Enable and reset TX and RX FIFO.
	if (install_irq(UART3_INT, (void *) UART3Handler, HIGHEST_PRIORITY) == FALSE) {
		return (FALSE);
	}

	U3IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART3 interrupt */
	return (TRUE);
}

// ќбработчик прерывани€ от UART3 (св€зь с блоком управлени€)
void UART3Handler(void) {
	BYTE IIRValue, LSRValue;
	BYTE Dummy = Dummy;
	BYTE dataReceive = 0;
	//IENABLE;				/* handles nested interrupt */
	IIRValue = U3IIR;

	IIRValue >>= 1; /* skip pending bit in IIR */
	IIRValue &= 0x07; /* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS){ /* Receive Line Status */
		LSRValue = U3LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			UART3Status = LSRValue;
			Dummy = U3RBR; /* Dummy read on RX to clear
			 interrupt, then bail out */
			//IDISABLE;
			VICVectAddr = 0; /* Acknowledge Interrupt */
			return;
		}
		if (LSRValue & LSR_RDR){ /* Receive Data Ready */
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			dataReceive = 1;
		}
	}else if (IIRValue == IIR_RDA){ /* Receive Data Available */
		dataReceive = 1;
	}else if (IIRValue == IIR_CTI){ /* Character timeout indicator */
		/* Character Time-out indicator */
		UART3Status |= 0x100; /* Bit 9 as the CTI error */
	}else if (IIRValue == IIR_THRE){ /* THRE, transmit holding register empty */
		/* THRE interrupt */
		LSRValue = U3LSR; /* Check status in the LSR to see if
		 valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UART3TxEmpty = 1;
			UART3TxCounter++;
			if (UART3TxCounter<UART3TxCount) U3THR = UART3TxBuffer[UART3TxCounter];
		}else {
			UART3TxEmpty = 0;
		}
	}

	if (dataReceive){
		UART3RxBuffer[UART3RxCounter] = U3RBR;
		UART3RxCounter++;

		switch (UART3RxCounter) {
			case 1:
				if (UART3RxBuffer[0] != 0xAA) UART3RxCounter = 0;
				break;
			case 2:
				if (UART3RxBuffer[1] != 0xBB)	UART3RxCounter = 0;
				break;
			default:
				if (UART3RxCounter >= IN_PACKET_SIZE) {
					if (!UART3dataIsReady){
						unsigned char i;
						for(i = 0; i<IN_PACKET_SIZE; i++) UART3readyData[i] = UART3RxBuffer[i];
						UART3dataIsReady = 1;
						UART3RxCounter = 0;
					}
				}
				break;
		}

		if (UART3RxCounter >= IN_PACKET_SIZE) {
			UART3RxCounter = 0;
		}
	}

	VICVectAddr = 0;
}

unsigned short CRC16(const volatile unsigned char *nData, unsigned short wLength){
    static const unsigned short wCRCTable[] = {
       0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
       0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
       0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
       0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
       0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
       0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
       0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
       0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
       0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
       0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
       0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
       0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
       0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
       0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
       0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
       0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
       0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
       0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
       0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
       0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
       0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
       0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
       0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
       0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
       0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
       0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
       0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
       0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
       0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
       0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
       0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
       0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

    unsigned char nTemp;
    unsigned short wCRCWord = 0xFFFF;

    while (wLength--){
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord  ^= wCRCTable[nTemp];
    }
    return wCRCWord;
}

//обработка прин€тых данных от блока управлени€
void readDataFromBU(){
	unsigned short calcCRC = CRC16(UART3readyData,IN_PACKET_SIZE-2);
	unsigned short CRC = (unsigned short)UART3readyData[IN_PACKET_SIZE-2]<<8 | UART3readyData[IN_PACKET_SIZE-1];
	if (CRC == calcCRC) {
		timeUART3WaitPacket = 0;
		RxData.PvxO2 = UART3readyData[2];
		RxData.PvxHe = UART3readyData[3];
		RxData.Pm = UART3readyData[4] + (UART3readyData[5] << 8);
		RxData.O2f = UART3readyData[6];
		RxData.Tf = UART3readyData[7];
		RxData.V = UART3readyData[8] + (UART3readyData[9] << 8);
		RxData.F = UART3readyData[10];

		RxData.Alarms.Low_P_O2 = UART3readyData[11] & 0x01;//	0 бит - УЌизкое давление ќ2Ф
		RxData.Alarms.Hi_P_O2 = UART3readyData[11] & 0x02;//	1 бит - У¬ысокое давление ќ2Ф
		RxData.Alarms.Low_P_He = UART3readyData[11] & 0x04;//	2 бит - УЌизкое давление HeФ
		RxData.Alarms.Hi_P_He = UART3readyData[11] & 0x08;//	3 бит - У¬ысокое давление HeФ
		RxData.Alarms.Low_P_P_O2 = UART3readyData[11] & 0x40;//	6 бит - УЌизка€ концентраци€ %ќ2Ф
		RxData.Alarms.Hi_P_P_O2 = UART3readyData[11] & 0x80;//	7 бит - У¬ысока€ концконцентраци€ %ќ2Ф

		RxData.Alarms.Datch_O2_neispr = UART3readyData[12] & 0x01;	//0 бит - У«амените датчик %ќ2Ф
		RxData.Alarms.Datch_O2_calibr = UART3readyData[12] & 0x02;	//1 бит - У»дет калибровка датчика %ќ2Ф
		RxData.Alarms.Device_fail = UART3readyData[12] & 0x04;		//2 бит - Ујппарат неработоспособенФ
		RxData.Alarms.Ingalation_off = UART3readyData[12] & 0x08;	//3 бит - »нгал€ци€ остановлена
		RxData.Alarms.Dangerous_pressure_in_the_mask = UART3readyData[12] & 0x10;//4 бит - ќпасное давление в маске
		RxData.Alarms.APNOE = UART3readyData[12] & 0x20;			//5 бит - јѕЌќЁ
		RxData.Alarms.HighLeak = UART3readyData[12] & 0x40;		//6 бит - "∆елательно заменить датчик"
		RxData.Alarms.Datch_ULTRASONIC_error = UART3readyData[12] & 0x80;//7 бит - неисправность ультрозвукового датчика

		RxData.V_skor = UART3readyData[13] + (UART3readyData[14] << 8);

		RxData.alarmsFromBU = ((unsigned short)UART3readyData[12]<<8 | UART3readyData[11]);

		RxData.O2flow = UART3readyData[15] + (UART3readyData[16] << 8);
		RxData.Heflow = UART3readyData[17] + (UART3readyData[18] << 8);
		RxData.O2concEtalon = UART3readyData[19] + (UART3readyData[20] << 8);
		RxData.O2concFast = UART3readyData[21] + (UART3readyData[22] << 8);
		RxData.HeaterTemp = UART3readyData[23];

		RxData.ADC1_array[0] = UART3readyData[24] + (UART3readyData[25] << 8);
		RxData.ADC1_array[1] = UART3readyData[26] + (UART3readyData[27] << 8);
		RxData.ADC1_array[2] = UART3readyData[27] + (UART3readyData[29] << 8);
		RxData.ADC1_array[3] = UART3readyData[30] + (UART3readyData[31] << 8);
		RxData.ADC1_array[4] = UART3readyData[32] + (UART3readyData[33] << 8);
		RxData.ADC1_array[5] = UART3readyData[34] + (UART3readyData[35] << 8);
		RxData.ADC1_array[6] = UART3readyData[36] + (UART3readyData[37] << 8);
		RxData.ADC1_array[7] = UART3readyData[38] + (UART3readyData[39] << 8);

		RxData.ADC2_array[0] = UART3readyData[40] + (UART3readyData[41] << 8);
		RxData.ADC2_array[1] = UART3readyData[42] + (UART3readyData[43] << 8);
		RxData.ADC2_array[2] = UART3readyData[44] + (UART3readyData[45] << 8);
		RxData.ADC2_array[3] = UART3readyData[46] + (UART3readyData[47] << 8);
		RxData.ADC2_array[4] = UART3readyData[48] + (UART3readyData[49] << 8);
		RxData.ADC2_array[5] = UART3readyData[50] + (UART3readyData[51] << 8);
		RxData.ADC2_array[6] = UART3readyData[52] + (UART3readyData[53] << 8);
		RxData.ADC2_array[7] = UART3readyData[54] + (UART3readyData[55] << 8);
		versionBU = UART3readyData[56];
		flags1 = UART3readyData[57]+(UART3readyData[58]<<8);//передача флагов

		/*capnoState = UART3readyData[56];
		capnoConfig = UART3readyData[57];
		capnoErrors = UART3readyData[59]>>2;

		capnoCO2 = (unsigned short)UART3readyData[58] | (unsigned short)(UART3readyData[59] & 0x03)<<8;

		capnoFiCO2 = UART3readyData[60];
		capnoEtCO2 = UART3readyData[61];
		capnoFiCO2pr = UART3readyData[62];
		capnoEtCO2pr = UART3readyData[63];
		capnoRR = UART3readyData[64];
		capnoNoLine = (capnoState>>1) & 0x01;
		capnoOkkluz = (capnoState>>2) & 0x01;
		capnoApnoe = (capnoState>>3) & 0x01;
		capnoInspiration = (capnoState>>5) & 0x01;
		capnoCalibrZero = (capnoState>>6) & 0x01;
		capnoCalibrGain = (capnoState>>7) & 0x01;
		capnoOnState = capnoConfig & 0x01;
		capnoError1 = capnoErrors & 0x01;
		capnoError2 = (capnoErrors>>2) & 0x01;
		capnoError3 = (capnoErrors>>3) & 0x01;
		capnoCalibrError = (capnoErrors>>4) & 0x01;
		capnoNoConnection = (capnoErrors>>5) & 0x01;*/
	}
}

//отправка данных блоку управлени€
void sendDataToBU(){
	UART3TxBuffer[0] = 0xAA;
	UART3TxBuffer[1] = 0xBB;
	UART3TxBuffer[2] = data[0];	//FiO2
	UART3TxBuffer[3] = data[1];	//RestTemp
	UART3TxBuffer[4] = data[2];	//CPAP
	UART3TxBuffer[5] = data[3];	//Psupport
	UART3TxBuffer[6] = data[4];	//Ftrig
	UART3TxBuffer[7] = data[5];	//ETS
	UART3TxBuffer[8] = data[6];	//Pramp
	if ((Dev_st == dev_work) && (stAlarm==0)) UART3TxBuffer[9] = 1;
	else if (stAlarm) UART3TxBuffer[9] = rej_alarm;
	else if (Dev_st == dev_stop) UART3TxBuffer[9] = 0;
	UART3TxBuffer[10] = data[7]; 		 //backup enable
	UART3TxBuffer[11] = data[8];		 //apnoe time
	UART3TxBuffer[12] = data[9];		 //backup BR
	UART3TxBuffer[13] = data[10];		 //backup support
	UART3TxBuffer[14] = data[11];		 //backup I:E
	UART3TxBuffer[15] = 0;
	UART3TxBuffer[16] = 0;
	UART3TxBuffer[17] = 0;
	UART3TxBuffer[18] = 0;
	UART3TxBuffer[19] = 0;
	UART3TxBuffer[20] = 0;

	UART3TxBuffer[21] = poprArray[0][0] & 0xFF;
	UART3TxBuffer[22] = poprArray[0][0] >> 8;
	UART3TxBuffer[23] = poprArray[0][1] & 0xFF;
	UART3TxBuffer[24] = poprArray[0][1] >> 8;
	UART3TxBuffer[25] = poprArray[0][2] & 0xFF;
	UART3TxBuffer[26] = poprArray[0][2] >> 8;
	UART3TxBuffer[27] = poprArray[0][3] & 0xFF;
	UART3TxBuffer[28] = poprArray[0][3] >> 8;
	UART3TxBuffer[29] = poprArray[0][4] & 0xFF;
	UART3TxBuffer[30] = poprArray[0][4] >> 8;
	UART3TxBuffer[31] = poprArray[0][5] & 0xFF;
	UART3TxBuffer[32] = poprArray[0][5] >> 8;
	UART3TxBuffer[33] = poprArray[0][6] & 0xFF;
	UART3TxBuffer[34] = poprArray[0][6] >> 8;
	UART3TxBuffer[35] = poprArray[0][7] & 0xFF;
	UART3TxBuffer[36] = poprArray[0][7] >> 8;
	UART3TxBuffer[37] = poprArray[0][8] & 0xFF;
	UART3TxBuffer[38] = poprArray[0][8] >> 8;

	UART3TxBuffer[39] = poprArray[1][0] & 0xFF;
	UART3TxBuffer[40] = poprArray[1][0] >> 8;
	UART3TxBuffer[41] = poprArray[1][1] & 0xFF;
	UART3TxBuffer[42] = poprArray[1][1] >> 8;
	UART3TxBuffer[43] = poprArray[1][2] & 0xFF;
	UART3TxBuffer[44] = poprArray[1][2] >> 8;
	UART3TxBuffer[45] = poprArray[1][3] & 0xFF;
	UART3TxBuffer[46] = poprArray[1][3] >> 8;
	UART3TxBuffer[47] = poprArray[1][4] & 0xFF;
	UART3TxBuffer[48] = poprArray[1][4] >> 8;
	UART3TxBuffer[49] = poprArray[1][5] & 0xFF;
	UART3TxBuffer[50] = poprArray[1][5] >> 8;
	UART3TxBuffer[51] = poprArray[1][6] & 0xFF;
	UART3TxBuffer[52] = poprArray[1][6] >> 8;
	UART3TxBuffer[53] = poprArray[1][7] & 0xFF;
	UART3TxBuffer[54] = poprArray[1][7] >> 8;
	UART3TxBuffer[55] = poprArray[1][8] & 0xFF;
	UART3TxBuffer[56] = poprArray[1][8] >> 8;
	UART3TxBuffer[57] = capnoGetControlByte();
	UART3TxBuffer[58] = capnoCalibrGaz;
	if (capnoCalibrOn){
		UART3TxBuffer[59] |= 0x80;
	}

	unsigned short calcCRC = CRC16(UART3TxBuffer,OUT_PACKET_SIZE-2);
	UART3TxBuffer[OUT_PACKET_SIZE-2] = calcCRC >> 8;
	UART3TxBuffer[OUT_PACKET_SIZE-1] = calcCRC;

	UART3TxCount = OUT_PACKET_SIZE;
	UART3TxCounter = 0;
	U3THR = UART3TxBuffer[0];
}


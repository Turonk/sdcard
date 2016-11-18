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
#include "blackbox.h"

volatile DWORD UART0Status;
volatile BYTE UART0TxEmpty = 1;
volatile BYTE UART0RxBuffer[BUFSIZE];
volatile BYTE UART0RxCount = 0;
volatile unsigned char priem_mas0 = 0;
volatile unsigned char UART0TxBuffer[255];
volatile unsigned char UART0TxCount = 0;
volatile unsigned char UART0TxCounter = 0;

volatile DWORD UART3Status;
volatile BYTE UART3TxEmpty = 1;
volatile BYTE UART3Buffer[BUFSIZE];
volatile BYTE UART3Count = 0;
volatile unsigned char priem_mas3 = 0;
volatile unsigned char timeUART3WaitPacket = 0;	//врем€ от предыдущего макета от системы управлени€
volatile unsigned short counter_old_datch = 0;
volatile unsigned short getPocketOk=0;

extern RTCTime Real_timer;
extern volatile unsigned char Timer_PC;
extern unsigned short data[8];
extern short poprArray[2][10];
extern SDeev Dev_st;
extern FRESULT f_getfree_result;
extern FRESULT disk_mount_result;
extern FRESULT file_open_result;
extern int f_printf_result;
extern DWORD fre_sect;
extern DWORD tot_sect;
extern unsigned short disk_file_count;
extern unsigned short num;
extern BYTE sendToPColdprot;
extern UINT fileByteRead;
extern UINT ByteToRead;
extern volatile char bufferRead[240];

/*
 * »нициализаци€ UART 0 или 3
 */
unsigned long UARTInit(unsigned long PortNum, unsigned long baudrate) {
	if (PortNum == 0) {
		RxData.RxDataTime.new = 0;
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
	}else if (PortNum == 3) {
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

		U3IER = IER_RBR/* | IER_THRE */| IER_RLS; /* Enable UART3 interrupt */
		return (TRUE);
	}
	return (FALSE);
}

/*
 * ‘ункци€ возвращает третий байт с флагами (дл€ св€зи и черного €щика)
 */
BYTE getByte_info_2(){
	BYTE res = 0;
	if (Dev_st == dev_work) res |= 1;//»Ќ√јЋя÷»я
	if (data[5] > 0) res |= 2; 	//подогрев
	if (st_KEY2 == 0) res |= 4; 	//ƒатчик подключен
	if (timeUART3WaitPacket>=5) res |= 8; //Ќет сигнала от блока управлени€
	return res;
}

/*
 * отправка данных на компьютер по протоколу старой программы мониторинга
 */
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

/*
 * ќбработчик прерывани€ от UART0 (св€зь с компьютером)
 */
void UART0Handler(void) {
	BYTE IIRValue, LSRValue;
	BYTE Dummy = Dummy;
	BYTE readByte = 0;

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
			readByte = 1;
		}
	}else if (IIRValue == IIR_RDA){ /* Receive Data Available */
		readByte = 1;
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

	if (readByte){
		BYTE rxbyte = U0RBR;
		switch (priem_mas0) {
			case 0:
				if (rxbyte == 0xAA) priem_mas0++;
				else priem_mas0 = 0;
				break;
			case 1:
				if (rxbyte == 0xBB) priem_mas0++;
				else priem_mas0 = 0;
				UART0RxCount = 0;
				break;
			case 2:
				UART0RxBuffer[UART0RxCount] = rxbyte;
				UART0RxCount++;
				if (UART0RxCount >= UART0RxBuffer[0]){
					if (UART0RxCount>2){
						BYTE i = 0;
						BYTE contrsumm = 0;
						for (i = 0; i < UART0RxCount - 1; i++) contrsumm += UART0RxBuffer[i];
						if (contrsumm == UART0RxBuffer[UART0RxCount - 1]){
							getPocketOk=1;	//переменна€ прин€ти€ пакета =1
						}
					}
					priem_mas0 = 0;
					UART0RxCount = 0;
				}
				break;
		}
	}
	// IDISABLE;
	VICVectAddr = 0; /* Acknowledge Interrupt */
}

void dataHandling(){
	getPocketOk=0;
	//переменна€ прин€ти€ пакета в 0
	BYTE rxCommand = UART0RxBuffer[1];
	BYTE contrsumm = 0;
	BYTE i = 0;
	if (rxCommand == 1 || rxCommand == 7){
		BYTE packetSize = 56;
		UART0TxBuffer[3] = 0x01;
		if (rxCommand == 7){
			packetSize = 24;
			UART0TxBuffer[3] = 0x07;
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
		UART0TxBuffer[20] = data[4];
		UART0TxBuffer[21] = data[5];
		UART0TxBuffer[22] = RxData.byte_alarms;
		UART0TxBuffer[23] = RxData.byte_info;
		UART0TxBuffer[24] = getByte_info_2();

		if (rxCommand == 1){
			unsigned char i;
			unsigned char arrconter = 25;
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
		Real_timer = RTCGetTime();
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
		Real_timer.RTC_Hour = UART0RxBuffer[2];
		Real_timer.RTC_Min = UART0RxBuffer[3];
		Real_timer.RTC_Sec = UART0RxBuffer[4];
		Real_timer.RTC_Mday = UART0RxBuffer[5];
		Real_timer.RTC_Mon = UART0RxBuffer[6];
		Real_timer.RTC_Year = (UART0RxBuffer[8]<<8) + UART0RxBuffer[7];
		Real_timer.RTC_Wday = UART0RxBuffer[9];
		Real_timer.RTC_Yday = (UART0RxBuffer[11]<<8) + UART0RxBuffer[10];
		RTCSetTime(Real_timer);

		Timer_PC = 0;
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

		Timer_PC = 0;
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
		sendToPColdprot = 1;
	}else if (rxCommand == 9){
		sendToPColdprot = 0;
	}else if (rxCommand == 10){

	}else if (rxCommand == 12){
		unsigned short N = (UART0RxBuffer[3]<<8) + UART0RxBuffer[2];
		FILINFO fno;
		readFileName(&fno, N);
		BYTE packetSize = 7;
		UART0TxBuffer[0] = 0xAA;
		UART0TxBuffer[1] = 0xBB;
		UART0TxBuffer[3] = 12;
		UART0TxBuffer[4] = fno.fsize;
		UART0TxBuffer[5] = fno.fsize>>8;
		UART0TxBuffer[6] = fno.fsize>>16;
		UART0TxBuffer[7] = fno.fsize>>24;

		int i=-1;
		do{
			i++;
			UART0TxBuffer[8+i] = fno.fname[i];
			packetSize++;
		}while((fno.fname[i]!=0) && (i<60));
		UART0TxBuffer[2] = packetSize;
		contrsumm = 0;
		for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
		UART0TxBuffer[packetSize + 1] = contrsumm;
		UART0TxCount = packetSize + 2;
		UART0TxCounter = 0;
		U0THR = UART0TxBuffer[UART0TxCounter];
	}else if (rxCommand == 13){
		unsigned short fileReadNumberElements = (UART0RxBuffer[3]<<8) + UART0RxBuffer[2];
		unsigned int fileReadDisloc = (UART0RxBuffer[7]<<24) + (UART0RxBuffer[6]<<16) + (UART0RxBuffer[5]<<8) + UART0RxBuffer[4];
		readLogFile(fileReadDisloc, fileReadNumberElements);

		BYTE packetSize = 7;
		UART0TxBuffer[0] = 0xAA;
		UART0TxBuffer[1] = 0xBB;
		UART0TxBuffer[3] = 13;
		UART0TxBuffer[4] = fileByteRead;
		UART0TxBuffer[5] = fileByteRead>>8;
		UART0TxBuffer[6] = fileByteRead>>16;
		UART0TxBuffer[7] = fileByteRead>>24;

		if (fileByteRead<=240){
			int i;
			for (i = 0; i<fileByteRead; i++){
				UART0TxBuffer[8+i] = bufferRead[i];
				packetSize++;
			}
		}

		UART0TxBuffer[2] = packetSize;
		contrsumm = 0;
		for (i = 2; i < packetSize + 1; i++) contrsumm += UART0TxBuffer[i];
		UART0TxBuffer[packetSize + 1] = contrsumm;
		UART0TxCount = packetSize + 2;
		UART0TxCounter = 0;
		U0THR = UART0TxBuffer[UART0TxCounter];
	}
}


/*
 * ќбработчик прерывани€ от UART3 (св€зь с блоком управлени€)
 */
void UART3Handler(void) {
	BYTE IIRValue, LSRValue;
	BYTE Dummy = Dummy;

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
		if (LSRValue & LSR_RDR) /* Receive Data Ready */
		{
			/* If no error on RLS, normal ready, save into the data buffer. */
			/* Note: read RBR will clear the interrupt */
			UART3Buffer[UART3Count] = U3RBR;
			UART3Count++;
			if (UART3Count == BUFSIZE) {
				UART3Count = 0; /* buffer overflow */
			}
		}
	}else if (IIRValue == IIR_RDA){ /* Receive Data Available */
		/* Receive Data Available */
		UART3Buffer[UART3Count] = U3RBR;
		UART3Count++;
		if (UART3Count == BUFSIZE) {
			UART3Count = 0; /* buffer overflow */
		}
	}else if (IIRValue == IIR_CTI){ /* Character timeout indicator */
		/* Character Time-out indicator */
		UART3Status |= 0x100; /* Bit 9 as the CTI error */
	}else if (IIRValue == IIR_THRE){ /* THRE, transmit holding register empty */
		/* THRE interrupt */
		LSRValue = U3LSR; /* Check status in the LSR to see if
		 valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UART3TxEmpty = 1;
		}else {
			UART3TxEmpty = 0;
		}
	}
	switch (priem_mas3) {
		case 0:
			if (UART3Buffer[UART3Count - 1] == 0xAA) priem_mas3++;
			else priem_mas3 = 0;
			break;
		case 1:
			if (UART3Buffer[UART3Count - 1] == 0xBB) {
				priem_mas3++;
				UART3Count = 0;
			}else priem_mas3 = 0;
			break;
		case 2:
			if (UART3Count >= 55) {
				BYTE contrsumm = 0;
				unsigned char i = 0;
				for (i = 0; i < 54; i++) contrsumm += UART3Buffer[i];
				if (UART3Buffer[54] == contrsumm){ //пакет прин€т правильно
					timeUART3WaitPacket = 0;
					RxData.PvxO2 = UART3Buffer[0];
					RxData.PvxHe = UART3Buffer[1];
					RxData.Pm = UART3Buffer[3];
					RxData.O2f = UART3Buffer[4];
					RxData.Tf = UART3Buffer[5];
					RxData.V = UART3Buffer[6] + (UART3Buffer[7] << 8);
					RxData.F = UART3Buffer[8];
					RxData.byte_alarms = UART3Buffer[9];
					RxData.byte_info = UART3Buffer[10];
					RxData.Alarms.Low_P_O2 = UART3Buffer[9] & 0x01;//	0 бит - УЌизкое давление ќ2Ф
					RxData.Alarms.Hi_P_O2 = UART3Buffer[9] & 0x02;//	1 бит - У¬ысокое давление ќ2Ф
					RxData.Alarms.Low_P_He = UART3Buffer[9] & 0x04;//	2 бит - УЌизкое давление HeФ
					RxData.Alarms.Hi_P_He = UART3Buffer[9] & 0x08;//	3 бит - У¬ысокое давление HeФ
					RxData.Alarms.Low_P_Xe = UART3Buffer[9] & 0x10;//	4 бит - УЌизкое давление XeФ
					RxData.Alarms.Hi_P_Xe = UART3Buffer[9] & 0x20;//	5 бит - У¬ысокое давление XeФ
					RxData.Alarms.Low_P_P_O2 = UART3Buffer[9] & 0x40;//	6 бит - УЌизка€ концентраци€ %ќ2Ф
					RxData.Alarms.Hi_P_P_O2 = UART3Buffer[9] & 0x80;//	7 бит - У¬ысока€ концконцентраци€ %ќ2Ф

					RxData.Info.Datch_O2_neispr = UART3Buffer[10] & 0x01;//0 бит - У«амените датчик %ќ2Ф
					RxData.Info.Datch_O2_calibr = UART3Buffer[10] & 0x02;//	1 бит - У»дет калибровка датчика %ќ2Ф
					RxData.Info.Device_fail = UART3Buffer[10] & 0x04;//	2 бит - Ујппарат неработоспособенФ
					RxData.Alarms.Ingalation_off = UART3Buffer[10] & 0x08;// 3 бит »нгал€ци€ остановлена
					RxData.Alarms.Dangerous_pressure_in_the_mask = UART3Buffer[10] & 0x10;// 4 бит ќпасное давление в маске
					RxData.Alarms.APNOE = UART3Buffer[10] & 0x20;// 5 бит јѕЌќЁ
					RxData.Alarms.Datch_Old = UART3Buffer[10] & 0x40;// 6 бит "∆елательно заменить датчик"
					RxData.Alarms.Datch_ULTRASONIC_error = UART3Buffer[10] & 0x80;//неисправность ультрозвукового датчика

					if (RxData.Alarms.Datch_Old) {
						if (counter_old_datch <= 800) counter_old_datch++;
						else RxData.Alarms.Datch_Old = 0;
					}else counter_old_datch = 0;

					RxData.V_skor = UART3Buffer[11] + (UART3Buffer[12] << 8);
					RxData.rezerv1 = 0;
					RxData.trevogi_summ = RxData.Alarms.Low_P_O2 +
											RxData.Alarms.Hi_P_O2 +
											RxData.Alarms.Low_P_He +
											RxData.Alarms.Hi_P_He +
											RxData.Alarms.Low_P_P_O2 +
											RxData.Alarms.Hi_P_P_O2 +
											RxData.Info.Datch_O2_neispr +
											RxData.Info.Datch_O2_calibr +
											RxData.Info.Device_fail +
											RxData.Alarms.Ingalation_off +
											RxData.Alarms.Dangerous_pressure_in_the_mask +
											RxData.Alarms.APNOE +
											RxData.Alarms.Datch_Old +
											RxData.Alarms.Datch_ULTRASONIC_error;

					RxData.O2flow = UART3Buffer[13] + (UART3Buffer[14] << 8);
					RxData.Heflow = UART3Buffer[15] + (UART3Buffer[16] << 8);
					RxData.O2concEtalon = UART3Buffer[17] + (UART3Buffer[18] << 8);
					RxData.O2concFast = UART3Buffer[19] + (UART3Buffer[20] << 8);
					RxData.HeaterTemp = UART3Buffer[21];

					unsigned char i;
					for(i = 22; i < 38; i+=2 ){
						RxData.ADC1_array[i/2-11] = UART3Buffer[i] + (UART3Buffer[i+1] << 8);
					}

					for(i = 38; i < 54; i+=2 ){
						RxData.ADC2_array[i/2-19] = UART3Buffer[i] + (UART3Buffer[i+1] << 8);
					}

				}
				priem_mas3 = 0;
			}
	}

	//IDISABLE;
	VICVectAddr = 0; /* Acknowledge Interrupt */
}

/*
 * ќтправка данных
 */
void UARTSend(unsigned long portNum, unsigned char *BufferPtr, unsigned long Length) {
	if (portNum == 0) {
		while (Length != 0) {
			/* THRE status, contain valid data */
			while ((U0LSR & 0x20) == 0);
			U0THR = *BufferPtr;
			UART0TxEmpty = 0; /* not empty in the THR until it shifts out */
			BufferPtr++;
			Length--;
		}
	}else if (portNum == 3) {
		while (Length != 0) {
			/* THRE status, contain valid data */
			while ((U3LSR & 0x20) == 0);
			U3THR = *BufferPtr;
			// UART3TxEmpty = 0;	/* not empty in the THR until it shifts out */
			BufferPtr++;
			Length--;
		}
	}
}

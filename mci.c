#include "LPC24xx.h"
#include "type.h"
#include "irq.h"
#include "mci.h"
#include <string.h>
#include "diskio.h"




/* --- MCI configurations --- */
#define N_BUF		4			/* Block transfer FIFO depth (>= 2) */
#define PCLK		72000000UL	/* PCLK supplied to MCI module */
#define MCLK_ID		400000UL	/* MCICLK for ID state (100k-400k) */
#define MCLK_RW		800000UL	/* MCICLK for data transfer (PCLK divided by even number) */

/* This MCI driver assumes that MCLK_RW is CCLK/4 or slower. If block buffer underrun/overrun
 /  occured due to any interrupt by higher priority process or slow external memory, increasing
 /  N_BUF or decreasing MCLK_RW will solve it. */

/* ----- Port definitions ----- */
#define	MMC_CD		1 /*!(FIO0PIN2 & 0x20)*/	/* Card detect (yes:true, no:false, default:true) */
#define	MMC_WP		0				 	/* Write protected (yes:true, no:false, default:false) */

/* ----- MMC/SDC command ----- */
#define CMD0	(0)				/* GO_IDLE_STATE */
#define CMD1	(1)				/* SEND_OP_COND (MMC) */
#define CMD2	(2)				/* ALL_SEND_CID */
#define CMD3	(3)				/* SEND_RELATIVE_ADDR */
#define ACMD6	(6|0x80)		/* SET_BUS_WIDTH (SDC) */
#define CMD7	(7)				/* SELECT_CARD */
#define CMD8	(8)				/* SEND_IF_COND */
#define CMD9	(9)				/* SEND_CSD */
#define CMD10	(10)			/* SEND_CID */
#define CMD12	(12)			/* STOP_TRANSMISSION */
#define CMD13	(13)			/* SEND_STATUS */
#define ACMD13	(13|0x80)		/* SD_STATUS (SDC) */
#define CMD16	(16)			/* SET_BLOCKLEN */
#define CMD17	(17)			/* READ_SINGLE_BLOCK */
#define CMD18	(18)			/* READ_MULTIPLE_BLOCK */
#define	CMD23	(23)			/* SET_BLK_COUNT (MMC) */
#define	ACMD23	(23|0x80)		/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)			/* WRITE_BLOCK */
#define CMD25	(25)			/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)			/* ERASE_ER_BLK_START */
#define CMD33	(33)			/* ERASE_ER_BLK_END */
#define CMD38	(38)			/* ERASE */
#define	ACMD41	(41|0x80)		/* SEND_OP_COND (SDC) */
#define CMD55	(55)			/* APP_CMD */

static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */
static volatile WORD Timer[4]; /* 1000Hz decrement timer for Transaction and Command */
volatile WORD CardRCA; /* Assigned RCA */
volatile BYTE CardType; /* Card type flag */
static BYTE CardInfo[16 + 16 + 4]; /* CSD(16), CID(16), OCR(4) */

/* Установка частоты тактирования карты */
#define MCI_PCLK					70325169	//CPU_CLOCK
#define MCI_CLK_ENABLE              ((unsigned long)1 << 8)
#define MCI_CLK_PWRSAVE             ((unsigned long)1 << 9)

volatile BYTE MCI_Block_End_Flag = 0;

/* Assembly modules using the ARM block transfer instruction in readfifo.s. */
extern int MCI_ReadFifo(volatile unsigned int * dest);
extern int MCI_WriteFifo(volatile unsigned int * dest);

volatile BYTE BufferBlock[512] __attribute__ ((aligned (4)));
/* treat WriteBlock as a constant address */

volatile DWORD TXBlockCounter = 0, RXBlockCounter = 0;

/******************************************************************************
 ** Function name:               MCI_Interrupt related
 ** Descriptions:                MCI interrupt handler and related APIs
 ******************************************************************************/
void MCI_TXEnable(void) {
	MCI_MASK0 = ((FIFO_TX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_TX_INT_MASK)); /* FIFO TX interrupts only */
	MCI_MASK1 = ((FIFO_TX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_TX_INT_MASK)); /* FIFO TX interrupts only */
	return;
}

void MCI_TXDisable(void) {
	MCI_MASK0 &= ~((FIFO_TX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_TX_INT_MASK)); /* FIFO TX interrupts only */
	MCI_MASK1 &= ~((FIFO_TX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_TX_INT_MASK)); /* FIFO TX interrupts only */
	return;
}

void MCI_RXEnable(void) {
	MCI_MASK0 = ((FIFO_RX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_RX_INT_MASK)); /* FIFO RX interrupts only */
	MCI_MASK1 = ((FIFO_RX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_RX_INT_MASK)); /* FIFO RX interrupts only */
	return;
}

/*****************************************************************/

void MCI_RXDisable(void) {
	MCI_MASK0 &= ~((FIFO_RX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_RX_INT_MASK)); /* FIFO TX interrupts only */
	MCI_MASK1 &= ~((FIFO_RX_INT_MASK) | (DATA_END_INT_MASK) | (ERR_RX_INT_MASK)); /* FIFO TX interrupts only */
	return;
}

/******************************************************************************
 ** Function name:               MCI_DataErrorProcess
 ** Descriptions:                Called by MCI interrupt handler Process data error.
 ******************************************************************************/
volatile BYTE mciDataError;
void MCI_DataErrorProcess(void) {
	DWORD MCIStatus;

	MCIStatus = MCI_STATUS;
	if (MCIStatus & MCI_DATA_CRC_FAIL) {
		MCI_CLEAR = MCI_DATA_CRC_FAIL;
	}
	if (MCIStatus & MCI_DATA_TIMEOUT) {
		MCI_CLEAR = MCI_DATA_TIMEOUT;
	}
	if (MCIStatus & MCI_TX_UNDERRUN) {
		MCI_CLEAR = MCI_TX_UNDERRUN;
	}
	if (MCIStatus & MCI_RX_OVERRUN) {
		MCI_CLEAR = MCI_RX_OVERRUN;
	}
	if (MCIStatus & MCI_START_BIT_ERR) {
		MCI_CLEAR = MCI_START_BIT_ERR;
	}

	mciDataError = 1;

	return;
}

/******************************************************************************
 ** Function name:               MCI_DATA_END_InterruptService
 ** Descriptions:                Called by MCI interrupt handler
 **                              This is the last interrupt module processing
 **                      		the block write and read to and from the MM card.
 **                      		FIFO interrupts are also used when DMA is disabled
 **                              This routine simply clears the MCI_Block_End_Flag, and increments counters for debug
 ******************************************************************************/
void MCI_DATA_END_InterruptService(void) {
	DWORD MCIStatus;

	MCIStatus = MCI_STATUS;
	if (MCIStatus & MCI_DATA_END) {
		MCI_CLEAR = MCI_DATA_END;
		return;
	}

	if (MCIStatus & MCI_DATA_BLK_END) {
		MCI_CLEAR = MCI_DATA_BLK_END;
		//MCI_TXDisable();
		//MCI_RXDisable();
		MCI_Block_End_Flag = 0;
		return;
	}

	return;
}

/******************************************************************************
 ** Function name:       MCI_FIFOInterruptService
 ** Descriptions:        Called by MCI interrupt handler when using FIFO interrupts and DMA is disabled
 ******************************************************************************/
void MCI_FIFOInterruptService(void) {
	DWORD MCIStatus;
	MCIStatus = MCI_STATUS;

	if (MCIStatus & (FIFO_TX_INT_MASK)) {
		MCI_CLEAR = FIFO_TX_INT_MASK;

		/* if using TX_HALF_EMPTY remove one WriteFifo below */
		if (MCIStatus & MCI_TX_HALF_EMPTY) { /* empty is multiple of 512 block size */
			MCI_WriteFifo((volatile unsigned int *) &BufferBlock[TXBlockCounter]); /* write 8 words to fifo */
			TXBlockCounter += 32;
		}
		if (TXBlockCounter == BLOCK_LENGTH) { /* block complete */
			TXBlockCounter = 0;
			//MCI_MASK0 &= ~(FIFO_TX_INT_MASK);  /* disable FIFO int until next block write */
			//MCI_MASK1 &= ~(FIFO_TX_INT_MASK);
			/* wait for SD card to complete sending data i.e MCI_DATA_BLK_END interrupt */
		}
	}else if (MCIStatus & (FIFO_RX_INT_MASK)) {
		MCI_CLEAR = FIFO_RX_INT_MASK;

		if (MCIStatus & MCI_RX_HALF_FULL) { /* if using RX_HALF_FULL remove one ReadFIFO below */
			MCI_ReadFifo((volatile unsigned int *) &BufferBlock[RXBlockCounter]); /* read 8 words from fifo */
			RXBlockCounter += 32;
		}
		if (RXBlockCounter == BLOCK_LENGTH) { /* block complete */
			RXBlockCounter = 0;
		}
	}

	return;
}

/******************************************************************************
 ** Function name:               MCI_IRQHandler
 ** Descriptions:                MCI interrupt handler
 **                              The handler to handle the block data write and read not for the commands.
 ******************************************************************************/
void MCI_IRQHandler(void) __attribute__ ((interrupt ("IRQ")));
void MCI_IRQHandler(void) {
	DWORD MCI_Status;
	MCI_Status = MCI_STATUS;
	if (MCI_Status & DATA_ERR_INT_MASK) {
		MCI_DataErrorProcess();
	}

	if (MCI_Status & DATA_END_INT_MASK) {
		MCI_DATA_END_InterruptService();
	}

	if (MCI_Status & FIFO_INT_MASK) {
		MCI_FIFOInterruptService();
	}
	//MCI_CLEAR = 0x7FF;
	VICVectAddr = 0; /* Acknowledge Interrupt */
}

static void power_off(void) {
	MCI_MASK0 = 0;
	MCI_COMMAND = 0;
	MCI_DATA_CTRL = 0;
	MCI_POWER = 0;
	MCI_CLOCK = 0;
	Stat |= STA_NOINIT;
}

/* Send a command token to the card and receive a response */
/* Returns 1 when function succeeded otherwise returns 0 */
/* idx - Command index (bit[5..0]), ACMD flag (bit7) */
/* arg - Command argument */
/* rt - Expected response type. None(0), Short(1) or Long(2) */
/* *buff - Response return buffer */
static int send_cmd(UINT idx, DWORD arg, UINT rt, DWORD *buff) {
	if (idx & 0x80) { /* Send a CMD55 prior to the specified command if it is ACMD class */
		if (!send_cmd(CMD55, (DWORD) CardRCA << 16, 1, buff) || !(buff[0] & 0x00000020)) return 0; /* When CMD55 is faild, exit with error */
	}
	idx &= 0x3F; /* Mask out ACMD flag */

	//

	Timer[1] = 100;
	do { /* Wait while CmdActive bit is set */
		MCI_COMMAND = 0; /* Cancel to transmit command */
		MCI_CLEAR = 0x3FF/*0x0C5*/; /* Clear status flags */
		UINT s;
		for (s = 0; s < 10; s++)
			MCI_STATUS; /* Skip lock out time of command reg. */
	}while ((MCI_STATUS & 0x00800) && Timer[1]);

	MCI_ARGUMENT = arg; /* Set the argument into argument register */
	UINT mc = 0x400 | idx; /* Enable bit + index */
	if (rt == 1) mc |= 0x040; /* Set Response bit to reveice short resp */
	if (rt > 1) mc |= 0x0C0; /* Set Response and LongResp bit to receive long resp */
	MCI_COMMAND = mc; /* Initiate command transaction */

	Timer[1] = 10;
	for (;;) { /* Wait for end of the cmd/resp transaction */
		if (!Timer[1]) return 0;
		UINT s = MCI_STATUS; /* Get the transaction status */
		if (rt == 0) {
			if (s & 0x080) return 1; /* CmdSent */
		}else {
			if (s & 0x040) break; /* CmdRespEnd */
			if (s & 0x001) { /* CmdCrcFail */
				if (idx == 1 || idx == 12 || idx == 41) break;/* Ignore resp CRC error on CMD1/12/41 */
				return 0;
			}
			if (s & 0x004) return 0; /* CmdTimeOut */
		}
	}

	if (rt) {
		buff[0] = MCI_RESP0; /* Read the response words */
		if (rt == 2) {
			buff[1] = MCI_RESP1;
			buff[2] = MCI_RESP2;
			buff[3] = MCI_RESP3;
		}
	}

	return 1; /* Return with success */
}

/*-----------------------------------------------------------------------*/
/* Wait card ready Returns 1 when card is tran state, otherwise returns 0*/
/*-----------------------------------------------------------------------*/
static int wait_ready(WORD tmr) {
	DWORD rc;
	Timer[0] = tmr;
	while (Timer[0]) {
		if (send_cmd(CMD13, (DWORD) CardRCA << 16, 1, &rc) && ((rc & 0x01E00) == 0x00800)) break;
	}
	return Timer[0] ? 1 : 0;
}

/* Swap byte order */
static void bswap_cp(BYTE *dst, const DWORD *src) {
	DWORD d;
	d = *src;
	*dst++ = (BYTE) (d >> 24);
	*dst++ = (BYTE) (d >> 16);
	*dst++ = (BYTE) (d >> 8);
	*dst++ = (BYTE) (d >> 0);
}

unsigned char initsteps;
unsigned char cmd0res;
unsigned char cmd8res;
unsigned long cmd8_response;
unsigned char acmd41res;
unsigned long acmd41_response;
unsigned char resp_command;
unsigned int msistat;

void setStatNoInit() {
	Stat = STA_NOINIT;
}

/* Initialize Disk Drive */
DSTATUS MCI_initialize(void) {
	//if (Stat & STA_NODISK) return Stat;				// No card in the socket

	CardRCA = 0;
	initsteps = 0;
	BYTE ty;
	for (Timer[0] = 100; Timer[0];)
		;

	PCONP |= 1 << 28; // Enable MCI
	unsigned char clk_div = MCI_PCLK / (2 * 800000); // Определение коэффициента деления входной чатоты
	if (!(MCI_PCLK % (2 * 800000))) clk_div--; // Проверка остатка от деления
	MCI_CLOCK = MCI_CLK_ENABLE | clk_div /*| MCI_CLK_PWRSAVE*/; // Установка требуемой частоты
	// очистка предыдущих функций выводов  MCICLK MCICMD MCIDAT0, MCIDAT1 MCIDAT2 MCIPWR MCIDAT3
	PINSEL1 &= ~(((unsigned long) 3 << 6) | ((unsigned long) 3 << 8) | ((unsigned long) 3 << 12));
	PINSEL2 &= ~(((unsigned long) 3 << 14) | ((unsigned long) 3 << 22) | ((unsigned long) 3 << 10) | ((unsigned long) 3 << 24));
	// определение выводов для интерфейса
	PINSEL1 |= (((unsigned long) 2 << 6) | ((unsigned long) 2 << 8) | ((unsigned long) 2 << 12));
	PINSEL2 |= (((unsigned long) 2 << 14) | ((unsigned long) 2 << 22) | ((unsigned long) 2 << 10) | ((unsigned long) 2 << 24));
	for (Timer[0] = 2; Timer[0];)
		; // Ожидание установки требуемой частоты

	MCI_MASK0 = 0;
	MCI_MASK1 = 0;
	MCI_COMMAND = 0;
	MCI_DATA_CTRL = 0;

	install_irq(MCI_INT, (void *) MCI_IRQHandler, LOWEST_PRIORITY - 1); // Register interrupt handlers for MCI event

	MCI_POWER = 0x02; // Socket power on
	for (Timer[0] = 10; Timer[0];)
		; // 10ms
	MCI_POWER = 0x03; // Enable signals

	initsteps = 1;

	cmd0res = send_cmd(CMD0, 0, 0, NULL); //Put the card into idle state
	for (Timer[0] = 2; Timer[0];)
		;
	initsteps = 2;
	DWORD resp[4];
	cmd8res = send_cmd(CMD8, 0x1AA, 1, &cmd8_response);
	for (Timer[0] = 2; Timer[0];)
		;
	initsteps = 2;

	if (cmd8res && (cmd8_response & 0xFFF) == 0x1AA) { /* SDC Ver2 and The card can work at vdd range of 2.7-3.6V*/
		initsteps = 3;
		unsigned int errors = 0;
		unsigned int maxcicle = 100;
		do {
			maxcicle--;
			errors++;
			if (errors > 100) goto di_fail;
			for (Timer[0] = 10; Timer[0];)
				;
			initsteps = 3;
			acmd41res = send_cmd(ACMD41, 0x40020000, 1, &acmd41_response);
			resp_command = MCI_RESP_CMD;
			msistat = MCI_STATUS;
		}while ((!acmd41res || !(acmd41_response & 0x80000000)) && maxcicle);
		if (maxcicle==0) goto di_fail;
		initsteps = 4;
		ty = (acmd41_response & 0x40000000) ? CT_SD2 | CT_BLOCK : CT_SD2; /* Check CCS bit in the OCR */
	}else { /* SDC Ver1 or MMC */
		initsteps = 4;
		UINT cmd;
		if (send_cmd(ACMD41, 0x00FF8000, 1, resp)) {
			ty = CT_SD1;
			cmd = ACMD41; /* ACMD41 is accepted -> SDC Ver1 */
		}else {
			ty = CT_MMC;
			cmd = CMD1; /* ACMD41 is rejected -> MMC */
		}
		unsigned int maxcicle = 100;
		do { /* Wait while card is busy state (use ACMD41 or CMD1) */
			/* This loop will take a time. Insert task rotation here for multitask envilonment. */
			if (!Timer[0]) goto di_fail;
		}while ((!send_cmd(cmd, 0x00FF8000, 1, resp) || !(resp[0] & 0x80000000)) && maxcicle);
		if (maxcicle==0) goto di_fail;
		initsteps = 5;
	}

	CardType = ty; /* Save card type */
	bswap_cp(&CardInfo[32], resp); /* Save OCR */

	/*---- Card is 'ready' state ----*/
	initsteps = 6;
	if (!send_cmd(CMD2, 0, 2, resp)) goto di_fail; /* Enter ident state */
	UINT n;
	for (n = 0; n < 4; n++)
		bswap_cp(&CardInfo[n * 4 + 16], &resp[n]); /* Save CID */

	/*---- Card is 'ident' state ----*/
	initsteps = 7;
	if (ty & CT_SDC) { /* SDC: Get generated RCA and save it */
		if (!send_cmd(CMD3, 0, 1, resp)) goto di_fail;
		CardRCA = (WORD) (resp[0] >> 16);
		initsteps = 8;
	}else { /* MMC: Assign RCA to the card */
		if (!send_cmd(CMD3, 1 << 16, 1, resp)) goto di_fail;
		CardRCA = 1;
		initsteps = 9;
	}

	/*---- Card is 'stby' state ----*/
	initsteps = 10;
	if (!send_cmd(CMD9, (DWORD) CardRCA << 16, 2, resp)) goto di_fail; /* Get CSD and save it */
	for (n = 0; n < 4; n++)
		bswap_cp(&CardInfo[n * 4], &resp[n]);
	initsteps = 11;
	if (!send_cmd(CMD7, (DWORD) CardRCA << 16, 1, resp)) goto di_fail; /* Select card */
	initsteps = 12;

	/*---- Card is 'tran' state ----*/
	if (!(ty & CT_BLOCK)) { /* Set data block length to 512 (for byte addressing cards) */
		initsteps = 13;
		if (!send_cmd(CMD16, 512, 1, resp) || (resp[0] & 0xFDF90000)) goto di_fail;
	}

	initsteps = 14;

	if (ty & CT_SDC) { /* Set wide bus mode (for SDCs) */
		initsteps = 15;
		if (!send_cmd(ACMD6, 2, 1, resp) || (resp[0] & 0xFDF90000)) goto di_fail;
		/* Set wide bus mode of SDC */MCI_CLOCK |= 0x800; /* Set wide bus mode of MCI */
	}

	MCI_CLOCK = (MCI_CLOCK & 0xF00) | (MCI_PCLK / (2 * 8000000)) /*| MCI_CLK_PWRSAVE*/; /* Set MCICLK = MCLK_RW, power-save mode */

	Stat &= ~STA_NOINIT; /* Clear STA_NOINIT */
	return Stat;

	di_fail: power_off();
	Stat |= STA_NOINIT; /* Set STA_NOINIT */
	return Stat;
}

/* Get Disk Status                                                       */
DSTATUS MCI_status(void) {
	return Stat;
}

/* Read Sector(s)                                                        */
/* buff - Pointer to the data buffer to store read data */
/* sector - Start sector number (LBA) */
/* count - Sector count (1..127) */
DRESULT MCI_read(BYTE *buff, DWORD sector, UINT count) {
	DWORD resp = 0;
	UINT cmd;
	DWORD i = 0;
	DWORD DataCtrl = 0;

	if (count < 1 || count > 127) return RES_PARERR; /* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY; /* Check drive status */
	if (!(CardType & CT_BLOCK)) sector *= 512; /* Convert LBA to byte address if needed */
	if (!wait_ready(500)) return RES_ERROR; /* Make sure that card is tran state */

	RXBlockCounter = 0;
	MCI_CLEAR = 0x7FF;
	MCI_DATA_CTRL = 0;
	for (i = 0; i < 0x10; i++)
		;

	MCI_DATA_TMR = DATA_TIMER_VALUE;
	MCI_DATA_LEN = BLOCK_LENGTH;

	cmd = (count > 1) ? CMD18 : CMD17; /* Transfer type: Single block or Multiple block */
	if (!send_cmd(cmd, sector, 1, &resp) || (resp & 0xC0580000)) {
		MCI_RXDisable();
		return RES_ERROR;
	}

	DataCtrl |= ((1 << 0) | (1 << 1) | (DATA_BLOCK_LEN << 4));
	MCI_Block_End_Flag = 1;

	MCI_RXEnable();
	MCI_DATA_CTRL = DataCtrl;
	for (i = 0; i < 0x10; i++)
		;
	Timer[3] = 100;



	do {


		while (MCI_Block_End_Flag && Timer[3]) { /* Wait for block arrival */

		}


		if (!Timer[3]) {
			MCI_RXDisable();
			MCI_DATA_CTRL = 0;
			MCI_CLEAR = 0x7FF;
			return RES_ERROR;
		}

		unsigned short ic;
		for (ic = 0; ic < 512; ic++) {
			 buff[ic] = BufferBlock[ic];
		}

		count--;
		if (count) {
			buff += BLOCK_LENGTH;
		}
	}while (count);

	MCI_RXDisable();
	MCI_DATA_CTRL = 0;
	MCI_CLEAR = 0x7FF;

	if (count || cmd == CMD18) send_cmd(CMD12, 0, 1, &resp); /* Terminate to read if needed */

	return count ? RES_ERROR : RES_OK;
}

/* *buff - Pointer to the data to be written */
/* sector - Start sector number (LBA) */
/* count - Sector count (1..127) */
DRESULT MCI_write(const BYTE *buff, DWORD sector, UINT count) {

	//if (savedata) return 1;

	if (count < 1 || count > 127) return RES_PARERR; /* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY; /* Check drive status */
	if (Stat & STA_PROTECT) return RES_WRPRT; /* Check write protection */
	if (!(CardType & CT_BLOCK)) sector *= 512; /* Convert LBA to byte address if needed */
	if (!wait_ready(500)) return RES_ERROR; /* Make sure that card is tran state */

	DWORD resp = 0;
	UINT cmd;
	DWORD i;
	DWORD DataCtrl = 0;

	TXBlockCounter = 0;
	MCI_CLEAR = 0x7FF;
	MCI_DATA_CTRL = 0;
	for (i = 0; i < 0x10; i++)
		;
	MCI_DATA_TMR = DATA_TIMER_VALUE;
	MCI_DATA_LEN = BLOCK_LENGTH;

	if (count == 1) { /* Single block write */
		cmd = CMD24;
	}else { /* Multiple block write */
		cmd = (CardType & CT_SDC) ? ACMD23 : CMD23;
		if (!send_cmd(cmd, count, 1, &resp) || (resp & 0xC0580000)) { /* Preset number of blocks to write */
			return RES_ERROR;
		}
		cmd = CMD25;
	}

	if (!send_cmd(cmd, sector, 1, &resp) || (resp & 0xC0580000)) { /* Send a write command */
		return RES_ERROR;
	}

	unsigned short ic;
	for (ic = 0; ic < 512; ic++) {
		BufferBlock[ic] = buff[ic];
	}
	DataCtrl |= ((1 << 0) | (DATA_BLOCK_LEN << 4));
	MCI_Block_End_Flag = 1;
	MCI_TXEnable();
	MCI_DATA_CTRL = DataCtrl;
	for (i = 0; i < 0x10; i++)
		;
	Timer[2] = 500;

	do {
		while (MCI_Block_End_Flag && Timer[2]) {

		}

		if (!Timer[2]) {
			MCI_TXDisable();
			MCI_DATA_CTRL = 0;
			return RES_ERROR;
		}

		count--;
		if (count) {/* Next user buffer address */
			buff += BLOCK_LENGTH;

			unsigned short ic;
			for (ic = 0; ic < 512; ic++) {
				BufferBlock[ic] = buff[ic];
			}
		}
	}while (count);

	MCI_TXDisable();
	MCI_DATA_CTRL = 0;

	if (count || (cmd == CMD25 && (CardType & CT_SDC))) send_cmd(CMD12, 0, 1, &resp);/* Terminate to write if needed */
	return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/* cmd - Control code 													 */
/* *buff - Buffer to send/receive data block */
/*-----------------------------------------------------------------------*/

DRESULT MCI_ioctl(BYTE cmd, void *buff) {
	DRESULT res;
	BYTE b, *ptr = buff, sdstat[64];
	DWORD resp[4], d, *dp, st, ed;
	static const DWORD au_size[] = { 1, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 24576, 32768, 49152, 65536, 131072 };

	if (Stat & STA_NOINIT) return RES_NOTRDY;
	res = RES_ERROR;

	switch (cmd) {
		case CTRL_SYNC: /* Make sure that all data has been written on the media */
			if (wait_ready(500)) /* Wait for card enters tarn state */
			res = RES_OK;
			break;

		case GET_SECTOR_COUNT: /* Get number of sectors on the disk (DWORD) */
			if ((CardInfo[0] >> 6) == 1) { /* SDC CSD v2.0 */
				d = CardInfo[9] + ((WORD) CardInfo[8] << 8) + ((DWORD) (CardInfo[7] & 63) << 16) + 1;
				*(DWORD*) buff = d << 10;
			}else { /* MMC or SDC CSD v1.0 */
				b = (CardInfo[5] & 15) + ((CardInfo[10] & 128) >> 7) + ((CardInfo[9] & 3) << 1) + 2;
				d = (CardInfo[8] >> 6) + ((WORD) CardInfo[7] << 2) + ((WORD) (CardInfo[6] & 3) << 10) + 1;
				*(DWORD*) buff = d << (b - 9);
			}
			res = RES_OK;
			break;

		case GET_BLOCK_SIZE: /* Get erase block size in unit of sectors (DWORD) */
			if (CardType & CT_SD2) { /* SDC ver 2.00 */
				if (MCI_ioctl(MMC_GET_SDSTAT, sdstat)) break;
				*(DWORD*) buff = au_size[sdstat[10] >> 4];
			}else { /* SDC ver 1.XX or MMC */
				if (CardType & CT_SD1) /* SDC v1 */
				*(DWORD*) buff = (((CardInfo[10] & 63) << 1) + ((WORD) (CardInfo[11] & 128) >> 7) + 1) << ((CardInfo[13] >> 6) - 1);
				else /* MMC */
				*(DWORD*) buff = ((WORD) ((CardInfo[10] & 124) >> 2) + 1) * (((CardInfo[11] & 3) << 3) + ((CardInfo[11] & 224) >> 5) + 1);
			}
			res = RES_OK;
			break;

		case CTRL_TRIM: /* Erase a block of sectors */
			if (!(CardType & CT_SDC) || (!(CardInfo[0] >> 6) && !(CardInfo[10] & 0x40))) break; /* Check if sector erase can be applied to the card */
			dp = buff;
			st = dp[0];
			ed = dp[1];
			if (!(CardType & CT_BLOCK)) {
				st *= 512;
				ed *= 512;
			}
			if (send_cmd(CMD32, st, 1, resp) && send_cmd(CMD33, ed, 1, resp) && send_cmd(CMD38, 0, 1, resp) && wait_ready(30000)) res = RES_OK;
			break;

		case CTRL_POWER_OFF:
			power_off(); /* Power off */
			res = RES_OK;
			break;

		case MMC_GET_TYPE: /* Get card type flags (1 byte) */
			*ptr = CardType;
			res = RES_OK;
			break;

		case MMC_GET_CSD: /* Get CSD (16 bytes) */
			memcpy(buff, &CardInfo[0], 16);
			res = RES_OK;
			break;

		case MMC_GET_CID: /* Get CID (16 bytes) */
			memcpy(buff, &CardInfo[16], 16);
			res = RES_OK;
			break;

		case MMC_GET_OCR: /* Get OCR (4 bytes) */
			memcpy(buff, &CardInfo[32], 4);
			res = RES_OK;
			break;

		case MMC_GET_SDSTAT: /* Receive SD status as a data block (64 bytes) */
			if (CardType & CT_SDC) { /* SDC */
				if (wait_ready(500)) {
					/*ready_reception(1, 64);				// Ready to receive data blocks
					 if (send_cmd(ACMD13, 0, 1, resp)	// Start to read
					 && !(resp[0] & 0xC0580000)) {
					 while ((XferWp == 0) && !(XferStat & 0xC));
					 if (!(XferStat & 0xC)) {
					 Copy_al2un(buff, DmaBuff[0], 64);
					 res = RES_OK;
					 }
					 }*/
					res = RES_OK;
				}
			}
			break;

		default:
			res = RES_PARERR;
	}

	return res;
}

/* Device Timer Interrupt Procedure */
/* This function must be called in period of 1ms */
void MCI_timerproc(void) {
	if (Timer[0]) --Timer[0];
	if (Timer[1]) --Timer[1];
	if (Timer[2]) --Timer[2];
	if (Timer[3]) --Timer[3];
}


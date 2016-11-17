#include "ff.h"
#include "diskio.h"
#include "mci.h"
#include "rtc.h"
#include "uart.h"
#include "LPC24xx.h"
#include "board.h"
#include "string.h"
#include "timer.h"

static char* inttostr(unsigned int n, unsigned char len) {
	static char buf[20];
	int i = 0;
	for (i = 0; i < len; i++) {
		buf[i] = '0';
	}
	i = 0;
	do {
		buf[i++] = n % 10 + '0';
	}while ((n /= 10) > 0);
	buf[len] = 0;

	int j;
	char c;
	for (i = 0, j = len - 1; i < j; i++, j--) {
		c = buf[i];
		buf[i] = buf[j];
		buf[j] = c;
	}

	return buf;
}

static void insertToStr(char* s1, char* s2, unsigned char p) {
	unsigned char i = 0;
	while (s2[i]) {
		s1[i + p] = s2[i];
		i++;
	}
}

long disk_sector_count = 0;
WORD disk_sector_size = 0;
BYTE disk_sdc_type = 0;

char retstrf[10];

FRESULT disk_mount_result = FR_DISK_ERR;
FRESULT file_open_result = FR_DISK_ERR;
int f_printf_result;

DWORD fre_clust, fre_sect, tot_sect;

unsigned short disk_file_count;
extern DWORD uptime;

char dateStr[] = "00000000.log";
bool SD_in = false; 	//вставлена ли SD карта (прошли ли инициализация и первая запись на сд карту правильно)
FRESULT f_getfree_result ;
volatile char bufferRead[240];
UINT ByteToRead;    	//Количество байт для чтения
UINT fileByteRead;   	//Количество прочитанных байт
FATFS fs;
DIR dir;

extern unsigned char initsteps;

bool initBlackBox() {
	setStatNoInit();

	fs.fs_type = 0;
	disk_mount_result = f_mount(&fs, "", 1);

	if (disk_mount_result == FR_OK) {
		initsteps = 20;

       //пристваевает файлу название с датой
		RTCTime Real_timer = RTCGetTime();
		unsigned int rt_year = Real_timer.RTC_Year < 9999 ? (unsigned int) Real_timer.RTC_Year : 0;
		unsigned int rt_mon = Real_timer.RTC_Mon < 13 ? (unsigned int) Real_timer.RTC_Mon : 0;
		unsigned int rt_day = Real_timer.RTC_Mday < 32 ? (unsigned int) Real_timer.RTC_Mday : 0;
		unsigned int rt_hour = Real_timer.RTC_Hour < 25 ? (unsigned int) Real_timer.RTC_Hour : 0;
		unsigned int rt_min = Real_timer.RTC_Min < 61 ? (unsigned int) Real_timer.RTC_Min : 0;
		unsigned int rt_sec = Real_timer.RTC_Sec < 61 ? (unsigned int) Real_timer.RTC_Sec : 0;

		if (rt_year < 2015 || rt_mon == 0 || rt_day == 0) {
			insertToStr(dateStr, "no_date_", 0);
		}else {
			insertToStr(dateStr, inttostr(rt_year, 4), 0);
			insertToStr(dateStr, inttostr(rt_mon, 2), 4);
			insertToStr(dateStr, inttostr(rt_day, 2), 6);
		}

		//определяем свободное место на карте
		FATFS *fs2 = &fs;
		f_getfree_result = f_getfree("", &fre_clust, &fs2);
		if (f_getfree_result == FR_OK) {
			tot_sect = (fs2->n_fatent - 2) * fs2->csize;
			if (tot_sect < 1000) return 0;
			fre_sect = fre_clust * fs2->csize;
		}else return 0;

		initsteps = 30;

		//определяем количество файлов
		FRESULT f_opendir_result = f_opendir(&dir, "");
		disk_file_count = 0;
		if (f_opendir_result == FR_OK) {
			FILINFO fno;
			FRESULT f_readdir_result;
			do {
				f_readdir_result = f_readdir(&dir, &fno);
				disk_file_count++;
			}while (fno.fname[0] && (f_readdir_result == FR_OK));
			f_closedir(&dir);
		}

		//удаление половины файлов если карта заполнена на 4/5
		if (fre_sect < (tot_sect / 5)) {
			DWORD deleteCount = disk_file_count / 2;
			FRESULT f_opendir_result = f_opendir(&dir, "");
			if (f_opendir_result == FR_OK) {
				FILINFO fno;
				FRESULT f_readdir_result;
				do {
					f_readdir_result = f_readdir(&dir, &fno);
					if (fno.fname[0] && fno.fname[1] && fno.fname[2] && (f_readdir_result == FR_OK) && deleteCount) {
						f_unlink(fno.fname);
						deleteCount--;
					}
				}while (fno.fname[0] && (f_readdir_result == FR_OK) && deleteCount);
				f_closedir(&dir);
			}
		}

		//создание нового файла или открытие существующего
		FILINFO fno;
		FRESULT f_stat_result;
		f_stat_result = f_stat(dateStr, &fno);
		FIL fp;
		file_open_result = f_open(&fp, dateStr, FA_OPEN_ALWAYS | FA_WRITE);
		if (file_open_result == FR_OK) {
			initsteps = 40;
			if (f_stat_result == FR_NO_FILE) {
				disk_file_count++;
				f_puts("# time;P_O2, atm;P_He, atm;Flow O2, l/min;Flow He, l/min;FiO2 Env, %;FiO2 sens2, %;P mask, cm H2O;Tmask, gradC;Tnagr, gradC;V, ml;f, 1/min;Tzad, gradC;FiO2 zad, %;Flags;\r\n", &fp);
			}else {
				f_lseek(&fp, f_size(&fp));
			}
			f_printf_result = f_printf(&fp, "# POWERON %02d.%02d.%04d %02d:%02d:%02d\r\n", rt_day, rt_mon, rt_year, rt_hour, rt_min, rt_sec);
			f_sync(&fp);
			f_close(&fp);
		}
	}

	return ((disk_mount_result == FR_OK) && (file_open_result == FR_OK) && (f_printf_result > 0));
}

//чтение имени N-ого файла
void readFileName(FILINFO* fno, unsigned short N){
	DIR dir2;
	FRESULT res = f_opendir(&dir2, "");
	if (res != FR_OK) return;
	unsigned short num = 0;
	do {
		res = f_readdir(&dir2, fno);
		num++;
	}while (num<=N && (res == FR_OK));
	f_closedir(&dir2);
}

//чтение данных из файла
void readLogFile(unsigned int fileReadDisloc, unsigned short fileReadNumberElements){
	FRESULT res;
	FILINFO fno;
	FIL fileRead;
	DIR dir2;
	res = f_opendir(&dir2, "");
	unsigned short num = 0;
	if (res != FR_OK) return;
	do {
		res = f_readdir(&dir2, &fno);
		num++;
	}while (num<=fileReadNumberElements && (res == FR_OK));
	f_closedir(&dir2);
	res = f_open(&fileRead, fno.fname ,FA_OPEN_EXISTING|FA_READ);
	if (res == FR_OK){
		res = f_lseek(&fileRead, fileReadDisloc);
		ByteToRead=240;
		res = f_read(&fileRead, bufferRead, ByteToRead, &fileByteRead);
		if (res!=FR_OK) fileByteRead = 0;
	}
	f_close(&fileRead);
}

DWORD flags_prev = 0;
unsigned char saveDataToBlackBoxCallCount = 0;

extern DWORD uptime;
extern TRxData RxData;
extern unsigned short data[8];
extern short poprArray[2][10];

//запись лога на карту памяти
FRESULT saveDataToBlackBox() {
	FRESULT res = FR_DISK_ERR;
	if (disk_mount_result == FR_OK && fre_sect > 10000) {
		BYTE byte_info_2 = getByte_info_2();
		DWORD flags = (DWORD) poprArray[0][4] << 24 | (DWORD) byte_info_2 << 16 |
				(DWORD) RxData.byte_info << 8 | (DWORD) RxData.byte_alarms;
		saveDataToBlackBoxCallCount++;
		if ((Dev_st == dev_work) || (flags != flags_prev) || (saveDataToBlackBoxCallCount>60)){
			flags_prev = flags;
			saveDataToBlackBoxCallCount = 0;

			FIL fp;
			file_open_result = f_open(&fp, dateStr, FA_OPEN_ALWAYS | FA_WRITE);
			if (file_open_result == FR_OK){


				res = f_lseek(&fp, f_size(&fp));
				if (res == FR_OK){
					res = f_printf(&fp, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%ld\r\n", uptime / 1000, RxData.PvxO2,
							RxData.PvxHe, RxData.O2flow, RxData.Heflow, RxData.O2concEtalon, RxData.O2concFast, RxData.Pm,
							RxData.Tf, RxData.HeaterTemp, RxData.V, RxData.F, data[5], data[4], flags);
				}
				f_sync(&fp);
				f_close(&fp);
			}
		}
	}
	return res;
}

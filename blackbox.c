#include "ff.h"
#include "diskio.h"
#include "mci.h"
#include "rtc.h"
#include "uart.h"
#include "LPC24xx.h"
#include "board.h"
#include "timer.h"
#include "screens.h"

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

char dateStr[] = "00000000.log";
bool SD_in = false; //вставлена ли SD карта (прошли ли инициализация и первая запись на сд карту правильно)

FIL fp;
FATFS fs;
DIR dir;
bool write_propusk;

extern unsigned char initsteps;

bool initBlackBox() {
	setStatNoInit();
	write_propusk = false;

	fs.fs_type = 0;
	disk_mount_result = f_mount(&fs, "", 1);

	if (disk_mount_result == FR_OK) {
		initsteps = 20;

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
		FRESULT f_getfree_result = f_getfree("", &fre_clust, &fs2);
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
			}
		}

		//создание нового файла или открытие существующего
		FILINFO fno;
		FRESULT f_stat_result;
		f_stat_result = f_stat(dateStr, &fno);
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
		}
	}

	return ((disk_mount_result == FR_OK) && (file_open_result == FR_OK) && (f_printf_result > 0));
}

static DWORD uptime_prev;
/*static unsigned char PvxO2_prev;
 static unsigned char PvxHe_prev;
 static unsigned short O2concEtalon_prev;
 static unsigned short O2concFast_prev;
 static unsigned char Pm_prev;
 static unsigned char Tf_prev;
 static unsigned char HeaterTemp_prev;
 static unsigned short V_prev;
 static unsigned char F_prev;
 static unsigned char Tzad_prev;
 static unsigned char O2zad_prev;
 static DWORD flags_prev;*/


FRESULT saveDataToBlackBox() {
	FRESULT res = FR_DISK_ERR;
	BYTE byte_info_2 = (Dev_st==dev_work) | ((data[5]>0)<<1) | ((st_KEY2==0)<<2) | ((timeUART3WaitPacket>=5)<<3);

	DWORD flags = (DWORD) poprArray[0][4] << 24 | (DWORD) byte_info_2 << 16 | (DWORD) RxData.alarmsFromBU;
	if (disk_mount_result == FR_OK && file_open_result == FR_OK && fre_sect > 10000) {
		/*if (RxData.PvxO2!=PvxO2_prev || RxData.PvxHe!=PvxHe_prev || RxData.O2concEtalon!=O2concEtalon_prev || RxData.O2concFast!=O2concFast_prev ||
		 RxData.Pm!=Pm_prev || RxData.Tf!=Tf_prev || RxData.HeaterTemp!=HeaterTemp_prev || RxData.V!=V_prev || RxData.F!=F_prev ||
		 data[5]!=Tzad_prev || data[4]!=O2zad_prev || flags!=flags_prev){
		 */
		/*if (write_propusk){
		 res = f_printf(&fp, "%ld;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%ld\r\n", uptime_prev, PvxO2_prev, PvxHe_prev, O2concEtalon_prev,
		 O2concFast_prev, Pm_prev, Tf_prev, HeaterTemp_prev, V_prev, F_prev, Tzad_prev, O2zad_prev, flags);
		 write_propusk = false;
		 }*/

		res = f_printf(&fp, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%ld\r\n", uptime / 1000, RxData.PvxO2, RxData.PvxHe,
				RxData.O2flow, RxData.Heflow, RxData.O2concEtalon, RxData.O2concFast, RxData.Pm, RxData.Tf, RxData.HeaterTemp,
				RxData.V, RxData.F, data[1], data[0], flags);

		//(DWORD)capnoGetSet()<<24 | capnoState<<16 | capnoConfig<<8 | capnoErrors, capnoFiCO2, capnoEtCO2, capnoRR

		f_sync(&fp);

		/*	PvxO2_prev = RxData.PvxO2;
		 PvxHe_prev = RxData.PvxHe;
		 O2concEtalon_prev = RxData.O2concEtalon;
		 O2concFast_prev = RxData.O2concFast;
		 Pm_prev = RxData.Pm;
		 Tf_prev = RxData.Tf;
		 HeaterTemp_prev = RxData.HeaterTemp;
		 V_prev = RxData.V;
		 F_prev = RxData.F;
		 Tzad_prev = data[5];
		 O2zad_prev = data[4];
		 flags_prev = flags;
		 }else write_propusk = true;*/
	}
	uptime_prev = uptime;
	return res;
}

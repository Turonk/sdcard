#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include "ff.h"

extern bool SD_in;
extern FRESULT disk_mount_result;
extern FRESULT file_open_result;
extern int f_printf_result;
extern DWORD fre_sect;
extern DWORD tot_sect;
extern unsigned short disk_file_count;


bool initBlackBox();
FRESULT saveDataToBlackBox();

#endif /* BLACKBOX_H_ */

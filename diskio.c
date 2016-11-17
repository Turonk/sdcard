/*-----------------------------------------------------------------------*/
/* Glue functions for FatFs - MCI/NAND drivers            (C)ChaN, 2010  */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include "mci.h"

DSTATUS disk_initialize(BYTE drv){
	return MCI_initialize();
}

DSTATUS disk_status (BYTE drv){
	return MCI_status();
}

DRESULT disk_read (BYTE drv, BYTE* buff, DWORD lba,	UINT count){
	return MCI_read(buff, lba, count);
}

DRESULT disk_write (BYTE drv, const BYTE* buff,	DWORD lba, UINT count){
	return MCI_write(buff, lba, count);
}

DRESULT disk_ioctl (BYTE drv, BYTE cmd,	void* buff){
	return MCI_ioctl(cmd, buff);
}



#ifndef BLACKBOX_H_
#define BLACKBOX_H_

#include "ff.h"

bool initBlackBox();
FRESULT saveDataToBlackBox();
//void readFileName();
void readFileName(FILINFO* fno, unsigned short N);
void readLogFile(unsigned int disloc, unsigned short numberElement);

#endif /* BLACKBOX_H_ */

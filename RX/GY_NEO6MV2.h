#ifndef _GY_NEO6MV2_H_
#define _GY_NEO6MV2_H_

#include "TxData.h"

void uart_init(void);

//GPGGA fieldNumbers

#define LATITUDE          3
#define LATITUDE_DIR      4
#define LONGITUDE         5
#define LONGITUDE_DIR     6
#define FIX               7
#define SATELLITES_COUNT  8
#define ALTITUDE          10

//GPVTG fieldNumbers
#define BEARING           2
#define GROUND_SPEED_KMPH 8              


#endif /* _GY_NEO6MV2_H_ */
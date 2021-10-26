#include "i2c_nb.h"

#ifdef USE_LSM6DS3
  #include "lsm6ds3_test.h"
#endif
#ifdef USE_BMP280
  #include "bmp280_i2c.h"
#endif

void proc_4ms();
void proc_120ms();
void proc_timer0();
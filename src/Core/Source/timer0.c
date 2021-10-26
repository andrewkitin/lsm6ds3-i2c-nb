#include "timer0.h"

void proc_4ms()
{
#ifdef USE_LSM6DS3
  if (--t4_sec1 == 0)
  {
    t4_sec1 = imu_reload_tim;
#ifdef USE_LSM6DS3
    imu_reload();
#endif // USE_LSM6DS3

  }
#endif // USE_LSM6DS3
}

void proc_120ms()
{
#ifdef USE_BMP280
  if (--t4_60ms == 0)
  {
    t4_60ms = 150;
    bmp_get_status();
  }
#endif 
}

void proc_timer0()
{
  proc_4ms();
  proc_120ms();
}



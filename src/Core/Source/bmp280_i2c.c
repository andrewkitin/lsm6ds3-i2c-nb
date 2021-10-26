#include "bmp280_i2c.h" 

//i2c_read_reg(0, LSM6DS3_FIFO_STATUS3, &lsm6ds3_fifo_pattern, 1, basic_callback);


__IO uint8_t t4_60ms = 150;
__IO uint8_t bmp_pressure_ready = 0;
__IO uint8_t is_getting_pressure = 0;
__IO uint8_t is_getting_status = 0;

uint8_t bmp_id;
bmp280_config_t bmp280_cfg;
bmp280_status_t bmp280_stat;

uint8_t pressure_val[3];

uint8_t who_am_i_callback(void* p)
{
  prepare_callback(p);
} 

uint8_t bmp_init()
{
  bmp280_cfg.os_temp = 0;
  bmp280_cfg.os_pres = 0;
  bmp280_cfg.odr = 0;
  bmp280_cfg.filter = 0;
  bmp280_cfg.spi3w_en = 0;
}

uint8_t bmp_setup_callback(void* p)
{
  prepare_callback(p);
}

uint8_t bmp_setup()
{
  
#ifdef BMP_ELEVATOR_COUNTER_SETTINGS
  bmp280_cfg.os_pres = BMP280_OS_4X;
  bmp280_cfg.os_temp = BMP280_OS_1X;
  bmp280_cfg.odr = BMP280_ODR_125_MS;
  bmp280_cfg.filter = BMP280_FILTER_COEFF_4;
  bmp280_cfg.spi3w_en = 0;
  bmp280_cfg.mode = BMP280_NORMAL_MODE;
#endif
  i2c_write_reg(bmp_addr, BMP280_CTRL_MEAS_ADDR, (uint8_t *)&bmp280_cfg, 2, bmp_setup_callback);
  return 0; 
}

uint8_t bmp_get_status_callback(void* p)
{
  prepare_callback(p);
  bmp_pressure_ready = !bmp280_stat.im_update;
  is_getting_status = 0;
  t4_60ms = 150;
}

uint8_t bmp_get_status()
{
  if (!is_getting_status)
  {
    if(i2c_read_reg(bmp_addr, BMP280_STATUS_ADDR, (uint8_t *) &bmp280_stat, 1, bmp_get_status_callback))
    {
      
    }
    is_getting_status = 1;
  }
}

uint8_t bmp_get_pressure_callback(void* p)
{
  prepare_callback(p);
  //copy pressure to other buffer
  is_getting_pressure = 0;
  t4_60ms = 150;
}

uint8_t bmp_get_pressure()
{
  if (!is_getting_pressure)
  {
    if(i2c_read_reg(bmp_addr, BMP280_PRES_MSB_ADDR, pressure_val, 3, bmp_get_pressure_callback))
    {
      
    }
    is_getting_pressure = 1;
    return 1;
  }
  return 0;
}

uint8_t bmp_loop()
{
  if(bmp_pressure_ready)
  {
    bmp_get_pressure();
    bmp_pressure_ready = 0;
  }
}

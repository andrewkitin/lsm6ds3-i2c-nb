
#include "stm32f40x.h"
#include "i2c_nb.h"
#include "bmp280_defs.h"

//#define bmp_addr BMP280_I2C_ADDR_SEC
#define bmp_addr 0xEE
//#define bmp_addr 0xEC
#define BMP_ELEVATOR_COUNTER_SETTINGS

extern __IO uint8_t t4_60ms;

void update_bmp_timer();

uint8_t bmp_init();
uint8_t who_am_i_callback(void* p);
uint8_t bmp_setup_callback(void* p);
uint8_t bmp_setup();


uint8_t bmp_get_status_callback(void* p);
uint8_t bmp_get_status();
uint8_t bmp_get_pressure_callback(void* p);
uint8_t bmp_get_pressure();

uint8_t bmp_loop();
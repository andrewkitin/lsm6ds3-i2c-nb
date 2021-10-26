
#ifndef LSM6DS3_TEST_H
#define LSM6DS3_TEST_H

#include "stm32f40x.h"
#include "lsm6ds3_reg.h"
#include "i2c_nb.h"

//#define LEDS_TEST
#define READ_DATA_BY_REGISTERS
#define READ_DATA_BY_FIFO
#define FIFO_TEST
#define LEDS_FIFO_TEST
#define IMU_USE_INTERRUPTS
#define INT1_FIFO_THR
//#define FIFO_THRESHOLD 256


#define IMU_FIFO_ODR LSM6DS3_FIFO_833Hz
#define IMU_ODR LSM6DS3_XL_ODR_833Hz
#define IMU_ALIASING_FILTER LSM6DS3_ANTI_ALIASING_50Hz
#define IMU_G LSM6DS3_2g

#define imu_reload_tim 1000
#ifdef IMU_USE_INTERRUPTS
  #define INT1_XL_DRD
  #define INT1_FIFO_THR

  #ifdef INT1_FIFO_THR
    #define FIFO_THRESHOLD 120
  #endif
#endif

#define IMU_SOFTWARE_RESET

#define MAX_FIFO_DEPTH                  8192
#define TEMP_FIFO_BUFFER_LENGTH         256


typedef struct imu_fifo_buffer
{
  uint8_t imu_fifo_buffer[MAX_FIFO_DEPTH];
  uint16_t buf_head;
  uint16_t buf_tail;
  uint16_t count;
} imu_fifo_buffer;

extern __IO uint8_t imu_setup_complete;
extern __IO uint8_t imu_fifo_mode_on;
extern __IO uint8_t imu_accel_leds_test;
extern __IO uint16_t t4_sec1;

void imu_fifo_buffer_init(imu_fifo_buffer *p);
uint8_t imu_fifo_buffer_get(imu_fifo_buffer *p, uint8_t *buf, uint16_t len, uint16_t buffer_len);
uint8_t imu_fifo_buffer_add(imu_fifo_buffer *p, uint8_t *buf, uint16_t buffer_len);

uint8_t test_nonblocking();
uint8_t i2c_loop();
uint8_t test_callback(void* p);

void update_imu_timer();
void IMU_init();

#ifdef IMU_USE_INTERRUPTS
void imu_IRQ(); 
#endif 
void IMU_i2c_init();

#ifdef LEDS_TEST
void IMU_leds_init();
uint8_t imu_accel_test_led();
uint8_t imu_accel_test_led_CALLBACK(void* p);
#endif

uint8_t IMU_setup();
uint8_t IMU_setup_CALLBACK(void* p);

uint8_t check_settings();
uint8_t check_settings_CALLBACK(void* p);

uint8_t imu_fifo_read();
uint8_t imu_fifo_read_H_CALLBACK(void* p);
uint8_t imu_fifo_read_L_CALLBACK(void* p);
uint8_t imu_fifo_get_length_CALLBACK(void *p);
uint8_t imu_fifo_clear_buffer_CALLBACK(void *p);
uint8_t imu_fifo_get_length();
uint8_t imu_accel_loop();

void imu_reload();
#endif
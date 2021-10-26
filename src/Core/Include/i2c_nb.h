#ifndef I2C_NB_H
#define I2C_NB_H

#define I2C_BUFF_LEN                    20
#define I2C_ACK_ENABLE

#define DEBUG_PU
//#define DEBUG_STM3240G

#define USE_LSM6DS3
//#define USE_BMP280

#include "stm32f40x_i2c.h"
#include "stm32f40x.h"

typedef enum 
{
  IDLE_STATUS,
  START_STATUS,
  ADDRESSED_STATUS,
  TRANSMITTING_STATUS,
  RECEIVING_STATUS,
  FINISHED_STATUS
} i2c_status_t;

typedef enum
{
  WRITE_DATA,
  WRITE_REG,
  READ_DATA,
  READ_REG
} i2c_operation;


typedef struct i2c_message
{
  uint8_t device_addr;
  uint8_t reg_addr;
  uint8_t direction; 
  i2c_status_t status;
  uint8_t *buf;
  uint8_t len;
  uint8_t (*f_callback)(void* p);
} i2c_message;

typedef struct i2c_buffer
{
  i2c_message message_buf[I2C_BUFF_LEN];
  uint8_t buf_head;
  uint8_t buf_tail;
  uint8_t count;
  
} i2c_buffer;

extern i2c_buffer i2c_message_buffer;

void i2c_buffer_init(i2c_buffer *p);
uint8_t i2c_buffer_get(i2c_buffer *p, i2c_message **to);
uint8_t i2c_buffer_add(i2c_buffer *p, i2c_message *from);

void i2c_init();

int32_t i2c_read_reg(uint8_t device_addr, uint8_t reg, uint8_t *buf, uint16_t len, uint8_t (*f_cb)(void* ));
int32_t i2c_write_reg(uint8_t device_addr, uint8_t reg, uint8_t *buf, uint16_t len, uint8_t (*f_cb)(void* ));

uint8_t read_reg_callback(void* p);
uint8_t prepare_callback(void *p);
uint8_t basic_callback(void *p);

uint8_t i2c_loop();
void gpio_i2c_delay();
void i2c_sda_recover();

#endif
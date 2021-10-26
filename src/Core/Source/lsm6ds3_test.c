#include "lsm6ds3_test.h"

#define imu_addr LSM6DS3_I2C_ADD_L

#ifdef IMU_USE_INTERRUPTS
imu_fifo_buffer fifo_buffer;
uint8_t temp_fifo_buffer[FIFO_THRESHOLD*2]; 
#endif

uint8_t lsm6ds3_fifo_status1;
uint8_t lsm6ds3_fifo_status2;
uint16_t lsm6ds3_fifo_length;
uint8_t lsm6ds3_fifo_pattern;
uint8_t error_occured;


uint8_t accel_buffer_l[256];
uint8_t LED_X;
uint8_t LED_Y;
uint8_t LED_Z;

//SETTINGS
lsm6ds3_ctrl1_xl_t ctrl_reg;
lsm6ds3_ctrl3_c_t ctrl3_c;
lsm6ds3_ctrl9_xl_t ctrl9_xl_reg;
lsm6ds3_int1_ctrl_t int_ctrl_reg;
lsm6ds3_md1_cfg_t md1_cfg_reg; 
lsm6ds3_master_config_t master_config_reg;


#ifdef READ_DATA_BY_FIFO

lsm6ds3_fifo_ctrl5_t fifo_ctrl_reg;
lsm6ds3_fifo_ctrl3_t fifo_dec_reg;

#ifdef INT1_FIFO_THR
uint16_t imu_fifo_threshold;
#endif

#endif

/*--------------------------*/
__IO uint8_t imu_setup_complete = 0;
__IO uint8_t imu_fifo_mode_on = 0;

__IO uint8_t imu_accel_leds_test = 0;
__IO uint8_t imu_accel_leds_ready = 1;

__IO uint8_t imu_fifo_overflow = 0;
__IO uint8_t imu_fifo_ready = 0;

__IO uint8_t accel_data_ready = 0;

__IO uint8_t imu_length_request_complete = 0;
__IO uint8_t imu_irq_complete = 0;
__IO uint16_t t4_sec1 = imu_reload_tim;
/*--------------------------*/

void update_imu_timer()
{
  t4_sec1 = imu_reload_tim;
}

void imu_startup_delay()
{
  for (uint32_t i = 0; i < 400000; ++i) {};
}

void imu_fifo_buffer_init(imu_fifo_buffer *p)
{
  p->buf_head = 0;
  p->buf_tail = 0;
  p->count = 0;
}

uint8_t imu_fifo_buffer_get(imu_fifo_buffer *p, uint8_t *buf, uint16_t len, uint16_t buffer_len)
{
  if (p->count <= 0)
  {
    //buffer is empty
    return 1;
  }
  if (len == 0)
  {
    //add zero_length packet
    return 0;
  }
  
  if (p->count < len)
  {
    return 1;
  }
  
  if (len > buffer_len)
  {
    // не хватает переданного буфера для считывания
    return 1;
  }
  
  uint8_t bytes_written = 0;
  if ((p->buf_head + len) >= MAX_FIFO_DEPTH)
  {
    //if we across a right border of buffer
    uint16_t first_part_len = MAX_FIFO_DEPTH - p->buf_head;
    memcpy(&(buf[bytes_written]), &(p->imu_fifo_buffer[p->buf_head]), first_part_len);
    p->buf_head = 0;
    bytes_written += first_part_len;
    uint16_t second_part_len = len - first_part_len;
    memcpy(&(buf[bytes_written]), &(p->imu_fifo_buffer[p->buf_head]), second_part_len);
    bytes_written += second_part_len;
    p->count -= len;
    p->buf_head += second_part_len;
  }
  else
  {
    memcpy(&(buf[bytes_written]), &(p->imu_fifo_buffer[p->buf_head]), len);
    p->count -= len;
    p->buf_head += len;
  }
  return 0;
}

uint8_t imu_fifo_buffer_add(imu_fifo_buffer *p, uint8_t *buf, uint16_t buffer_len)
{
  if ((p->count + buffer_len) >= MAX_FIFO_DEPTH)
  {
    //buffer is full
    return 1;
  }
  
  if (buffer_len == 0)
  {
    //zero-buffer add
    return 0;
  }
  
  uint16_t bytes_written = 0;
  if ((p->buf_tail + buffer_len) >= MAX_FIFO_DEPTH)
  { 
    uint16_t first_part_len = MAX_FIFO_DEPTH - p->buf_tail;
    memcpy(&(p->imu_fifo_buffer[p->buf_tail]), &(buf[bytes_written]), first_part_len);
    bytes_written += first_part_len;
    p->buf_tail = 0;
    uint16_t second_part_len = buffer_len - first_part_len;
    memcpy(&(p->imu_fifo_buffer[p->buf_tail]), &(buf[bytes_written]), second_part_len);
    bytes_written += second_part_len;
    p->count += buffer_len;
    p->buf_tail += second_part_len;
  }
  else 
  {
    memcpy(&(p->imu_fifo_buffer[p->buf_tail]), &(buf[bytes_written]), buffer_len);
    p->count += buffer_len;
    p->buf_tail += buffer_len;
  }
  return 0;
}

uint8_t test_callback(void* p)
{
    i2c_message *msg = (i2c_message *)p;
    msg->status = FINISHED_STATUS;
    return 0;
}

uint8_t IMU_setup_CALLBACK(void* p)
{
   prepare_callback(p);
   update_imu_timer();
   imu_setup_complete = 1; 
#ifdef LEDS_TEST
   imu_accel_leds_test = 1; 
   imu_accel_leds_ready = 1;
#endif
   
#ifdef FIFO_TEST
   imu_fifo_mode_on = 1;
   imu_fifo_ready = 1;
#endif
   
   return 0;
}

uint8_t IMU_setup_complete()
{
    
   imu_setup_complete = 1; 
#ifdef LEDS_TEST
   imu_accel_leds_test = 1; 
   imu_accel_leds_ready = 1;
#endif
     
#ifdef FIFO_TEST
   imu_fifo_mode_on = 1;
   imu_fifo_ready = 1;
#endif
   return 0;
}

uint8_t imu_fifo_clear_buffer_CALLBACK(void *p)
{
  prepare_callback(p);
  update_imu_timer();
  //настройка дальнейшая буфера
  fifo_ctrl_reg.fifo_mode = LSM6DS3_STREAM_MODE;
  fifo_ctrl_reg.odr_fifo = IMU_FIFO_ODR;
  if(i2c_write_reg(imu_addr, LSM6DS3_FIFO_CTRL5, (uint8_t *)&fifo_ctrl_reg, 1, basic_callback)) return 1;
  return 0;
}

uint8_t IMU_setup()
{

#ifdef IMU_SOFTWARE_RESET
    ctrl3_c.sw_reset = 1;
    if(i2c_write_reg(imu_addr, LSM6DS3_CTRL3_C, (uint8_t *)&ctrl3_c, 1, basic_callback)) return 1;
    imu_startup_delay();
#endif //IMU_SOFTWARE_RESET
    
    // сброс буфера
    fifo_ctrl_reg.fifo_mode = LSM6DS3_BYPASS_MODE;
    fifo_ctrl_reg.odr_fifo = LSM6DS3_FIFO_DISABLE;
    if(i2c_write_reg(imu_addr, LSM6DS3_FIFO_CTRL5, (uint8_t *)&fifo_ctrl_reg, 1, imu_fifo_clear_buffer_CALLBACK)) return 1;
    imu_startup_delay();
    
    // Accel setup (2g, 833 Hz ODR, 400 Hz Anti-aliasing)
    ctrl_reg.fs_xl = LSM6DS3_2g;
    ctrl_reg.odr_xl = IMU_ODR;
    ctrl_reg.bw_xl = IMU_ALIASING_FILTER;
    if(i2c_write_reg(imu_addr, LSM6DS3_CTRL1_XL, (uint8_t *)&ctrl_reg, 1, basic_callback)) return 1;
    
    ctrl9_xl_reg.xen_xl = 1;
    ctrl9_xl_reg.yen_xl = 1;
    ctrl9_xl_reg.zen_xl = 1;
    ctrl9_xl_reg.not_used_01 = 0;
    ctrl9_xl_reg.not_used_02 = 0;
    ctrl9_xl_reg.soft_en = 0;
    if(i2c_write_reg(imu_addr, LSM6DS3_CTRL9_XL, (uint8_t *)&ctrl9_xl_reg, 1, basic_callback)) return 1;
    
    // FIFO setup () 
    // LSM6DS3_FIFO_CTRL5
    
     //fifo batch 
    fifo_dec_reg.dec_fifo_xl = 1;
    fifo_dec_reg.dec_fifo_gyro = 0;
    if(i2c_write_reg(imu_addr, LSM6DS3_FIFO_CTRL3, (uint8_t *)&fifo_dec_reg, 1, basic_callback)) return 1;
    
#ifdef IMU_USE_INTERRUPTS

    // INT1
#ifdef INT1_XL_DRDY
    int_ctrl_reg.int1_drdy_xl = 1; // Accel Data Ready
#elif defined(INT1_FIFO_THR)
    int_ctrl_reg.int1_fth = 1;
#endif
    if(i2c_write_reg(imu_addr, LSM6DS3_INT1_CTRL, (uint8_t *)&int_ctrl_reg, 1, basic_callback)) return 1;
    
#ifdef INT1_FIFO_THR
    imu_fifo_threshold = FIFO_THRESHOLD & 0x0FFF;
    if(i2c_write_reg(imu_addr, LSM6DS3_FIFO_CTRL1, (uint8_t *)&imu_fifo_threshold, 2, basic_callback)) return 1;
#endif
    
#endif    
    //MD1_CFG
    //md1_cfg_reg.int1_timer = 0;
    //md1_cfg_reg.int1_tilt = 0;
    //md1_cfg_reg.int1_6d = 0;
    //md1_cfg_reg.int1_double_tap = 0;
    //md1_cfg_reg.int1_ff = 0;
    //md1_cfg_reg.int1_wu = 0;
    //md1_cfg_reg.int1_single_tap = 0;
    //md1_cfg_reg.int1_inact_state = 0;
    //if(i2c_write_reg(0, LSM6DS3_MD1_CFG, (uint8_t *)&md1_cfg_reg, 1, write_reg_callback)) return 1;
    
    //master_config_reg.drdy_on_int1 = 0;
    //if(i2c_write_reg(0, LSM6DS3_MASTER_CONFIG, (uint8_t *)&master_config_reg, 1, write_reg_callback)) return 1;
    IMU_setup_complete();
    return 0;
    
}

#ifdef LEDS_TEST
void IMU_leds_init()
{

#ifdef DEBUG_STM3240G
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOI | RCC_AHB1Periph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOG, GPIO_Pin_6 | GPIO_Pin_8);
    GPIO_ResetBits(GPIOI, GPIO_Pin_9);
    GPIO_SetBits(GPIOC, GPIO_Pin_7);
    
#endif
    
#ifdef DEBUG_PU
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_7 | GPIO_Pin_9);
    //GPIO_ResetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_7 | GPIO_Pin_9);
    
#endif

}
#endif // LEDS_TEST

void IMU_i2c_init()
{
  
  
#ifdef LEDS_TEST
  IMU_leds_init();
#endif // LEDS_TEST
    
  
#ifdef IMU_USE_INTERRUPTS
  
#ifdef DEBUG_PU
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource15);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

  #endif // DEBUG_PU
   
  #ifdef DEBUG_STM3240G
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  #endif // DEBUG_STM3240G
    
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif //IMU_USE_INTERRUPTS    
}


void IMU_init()
{
    imu_startup_delay();
    IMU_i2c_init();

    lsm6ds3_fifo_status1 = 0;
    lsm6ds3_fifo_status2 = 0;
    lsm6ds3_fifo_length = 0;
    imu_length_request_complete = 1;
    error_occured = 0;
    
    memset(&ctrl_reg, 0, sizeof(ctrl_reg));
    memset(&ctrl3_c, 0, sizeof(ctrl3_c));
    memset(&ctrl9_xl_reg, 0, sizeof(ctrl9_xl_reg));
    memset(&fifo_ctrl_reg, 0, sizeof(fifo_ctrl_reg));
    memset(&int_ctrl_reg, 0, sizeof(int_ctrl_reg));
    memset(&md1_cfg_reg, 0, sizeof(md1_cfg_reg));
    memset(&fifo_dec_reg, 0, sizeof(fifo_dec_reg));
    memset(&master_config_reg, 0, sizeof(master_config_reg));
} 

#ifdef FIFO_TEST

uint8_t imu_fifo_get_length_CALLBACK(void *p)
{
  prepare_callback(p);
  update_imu_timer();
  uint8_t temp = 0x0F & lsm6ds3_fifo_status2;
  lsm6ds3_fifo_length = (temp << 8) | lsm6ds3_fifo_status1;
  imu_length_request_complete = 1;
  return 0;
}

uint8_t imu_fifo_get_length()
{
  if(imu_length_request_complete)
  {
    i2c_read_reg(imu_addr, LSM6DS3_FIFO_STATUS3, &lsm6ds3_fifo_pattern, 1, basic_callback);
    i2c_read_reg(imu_addr, LSM6DS3_FIFO_STATUS1, &lsm6ds3_fifo_status1, 1, basic_callback);
    i2c_read_reg(imu_addr, LSM6DS3_FIFO_STATUS2, &lsm6ds3_fifo_status2, 1, imu_fifo_get_length_CALLBACK);
    imu_length_request_complete = 0;
    return 0;
  }
  return 1;
}


uint8_t imu_fifo_read_L_CALLBACK(void* p)
{
  prepare_callback(p);
  update_imu_timer();
  //imu_fifo_ready = 1;
  if(!accel_data_ready)
  {
    imu_fifo_overflow = 1;
  }
#ifdef LEDS_FIFO_TEST
  imu_fifo_buffer_add(&fifo_buffer, temp_fifo_buffer, lsm6ds3_fifo_length);
#endif 
  return 0;
}

uint8_t imu_fifo_read_H_CALLBACK(void* p)
{
  prepare_callback(p);
  update_imu_timer();
  if(!accel_data_ready)
  {
    imu_fifo_overflow = 1;
  }
#ifdef LEDS_FIFO_TEST
  imu_fifo_buffer_add(&fifo_buffer, temp_fifo_buffer, FIFO_THRESHOLD);
  //lsm6ds3_fifo_length = 0;
#endif
  imu_fifo_ready = 1;
  return 0;
}


uint8_t imu_fifo_read()
{
  if(!imu_fifo_mode_on) return 1;
  
  
  if (imu_fifo_ready) 
  {
    
    if (lsm6ds3_fifo_length == 0)
    {
      imu_fifo_get_length();
    }
    else
    {
      if (1)
      {
        i2c_read_reg(imu_addr, LSM6DS3_FIFO_DATA_OUT_L, temp_fifo_buffer, lsm6ds3_fifo_length*2, imu_fifo_read_H_CALLBACK);
        imu_fifo_ready = 0;
      }
      else
      {
        lsm6ds3_fifo_length = 0;
      }
    }
    
  }
  
  if(accel_data_ready)
  {
    accel_data_ready = 0;
    //handle accel data here
    accel_data_ready = 1;
  }
  return 0;
}

#endif //FIFO_TEST

uint8_t imu_IRQ_callback(void* p)
{
   prepare_callback(p);
   update_imu_timer();
   return 0;
}

#ifdef IMU_USE_INTERRUPTS
void imu_IRQ()
{
#ifdef DEBUG_PU
  if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 1)
#elif defined(DEBUG_STM3240G)
  if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 1)
#endif
  {
    if (imu_fifo_ready)
    {
      t4_sec1 = imu_reload_tim;
      imu_fifo_get_length();
      if(i2c_read_reg(imu_addr, LSM6DS3_FIFO_DATA_OUT_L, temp_fifo_buffer, FIFO_THRESHOLD, imu_fifo_read_H_CALLBACK))
      {
        uint8_t debug = 0;
        return;
      }
      imu_fifo_ready = 0;
    }
    else
    {
      error_occured = 1;
    }
  }
  else 
  {
    error_occured = 1;
  }

#ifdef DEBUG_PU
  EXTI_ClearITPendingBit(EXTI_Line15);
#elif defined(DEBUG_STM3240G)
  EXTI_ClearITPendingBit(EXTI_Line13);
#endif
}
#endif

#ifdef LEDS_TEST
uint8_t update_leds(int16_t* axis)
{
  #ifdef DEBUG_PU
  if(axis[0] > 14000 || axis[0] < -14000)
  {
    GPIO_ResetBits(GPIOC, GPIO_Pin_1);
  }
  else
  {
    GPIO_SetBits(GPIOC, GPIO_Pin_1);
  }
  
  if(axis[1] > 16000 || axis[1] < -16000)
  {
    GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  }
  else
  {
    GPIO_SetBits(GPIOC, GPIO_Pin_7);
  }
  
  if(axis[2] > 16000 || axis[2] < -16000)
  {
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
  }
  else
  {
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
  }
  return 0;
  #endif
  
  #ifdef DEBUG_STM3240G
  if(axis[0] > 16000 || axis[0] < -16000)
  {
    GPIO_SetBits(GPIOG, GPIO_Pin_6);
  }
  else
  {
    GPIO_ResetBits(GPIOG, GPIO_Pin_6);

  }
  
  if(axis[1] > 14000 || axis[1] < -14000)
  {
    GPIO_SetBits(GPIOG, GPIO_Pin_8);
  }
  else
  {
    GPIO_ResetBits(GPIOG, GPIO_Pin_8);
  }
  
  if(axis[2] > 14000 || axis[2] < -14000)
  {
    GPIO_SetBits(GPIOI, GPIO_Pin_9);
  }
  else
  {
    GPIO_ResetBits(GPIOI, GPIO_Pin_9);
  }
  return 0;
  #endif
}


uint8_t reset(void* p)
{
  prepare_callback(p);
  imu_accel_leds_ready = 1;
  return 0;
}


uint8_t imu_accel_test_led_CALLBACK(void* p)
{
  prepare_callback(p);
  update_imu_timer();
  int16_t axis[3];
  axis[0] = (int16_t)accel_buffer_l[1];
  axis[0] = (axis[0] * 256) + (int16_t)accel_buffer_l[0];
  axis[1] = (int16_t)accel_buffer_l[3];
  axis[1] = (axis[1] * 256) + (int16_t)accel_buffer_l[2];
  axis[2] = (int16_t)accel_buffer_l[5];
  axis[2] = (axis[2] * 256) + (int16_t)accel_buffer_l[4];
  
  update_leds(axis);
  imu_accel_leds_ready = 1;
  return 0;
}

uint8_t imu_accel_test_led()
{
  if(imu_accel_leds_ready)
  {
    i2c_read_reg(imu_addr, LSM6DS3_OUTX_L_XL, accel_buffer_l, 6, imu_accel_test_led_CALLBACK);
    imu_accel_leds_ready = 0;
  }
  else return 1;
  return 0; 
}

uint8_t check_settings_CALLBACK(void* p)
{
  prepare_callback(p);
  update_imu_timer();
  imu_setup_complete = 0;
  return 0;
}
#endif // LEDS_TEST


void imu_reload()
{
  i2c_init();
  i2c_buffer_init(&i2c_message_buffer);
  IMU_init();
  IMU_setup();
}

uint8_t send_msg_callback(void* p)
{
  prepare_callback(p);
  update_imu_timer();
  return 0;
}

void IMU_send_msg()
{
  
  //i2c_write_reg(imu_addr, LSM6DS3_CTRL9_XL, (uint8_t *)&ctrl9_xl_reg, 1, send_msg_callback);
  i2c_read_reg(imu_addr, LSM6DS3_FIFO_STATUS3, &lsm6ds3_fifo_pattern, 1, send_msg_callback);
}

uint8_t imu_accel_loop()
{
  
#ifdef LEDS_FIFO_TEST
  int16_t axis[3];
  //uint8_t status = imu_fifo_buffer_get(&fifo_buffer, accel_buffer_l, 6, sizeof(accel_buffer_l));
  uint8_t status = imu_fifo_buffer_get(&fifo_buffer, accel_buffer_l, FIFO_THRESHOLD, sizeof(accel_buffer_l));
  uint8_t* idx = accel_buffer_l;
  if(!status)
  {
    if (lsm6ds3_fifo_pattern == 0)
    {
      axis[0] = (idx[1] << 8) | (idx[0]);
      axis[1] = (idx[3] << 8) | (idx[2]);
      axis[2] = (idx[5] << 8) | (idx[4]);
    }
    if (lsm6ds3_fifo_pattern == 1)
    {
      axis[1] = (idx[1] << 8) | (idx[0]);
      axis[2] = (idx[3] << 8) | (idx[2]);
      axis[0] = (idx[5] << 8) | (idx[4]);
    }
    
    if (lsm6ds3_fifo_pattern == 2)
    {
      axis[2] = (idx[1] << 8) | (idx[0]);
      axis[0] = (idx[3] << 8) | (idx[2]);
      axis[1] = (idx[5] << 8) | (idx[4]);
    }
    
    //imu_fifo_ready = 1;
    //lsm6ds3_fifo_length = 0;
    update_leds(axis);
  }
  else return 1;
  return 0;
#endif // LEDS FIFO TEST
}


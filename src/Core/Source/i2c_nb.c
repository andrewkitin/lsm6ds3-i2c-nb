#include "i2c_nb.h"

i2c_buffer i2c_message_buffer;

void gpio_i2c_delay()
{
  for (uint32_t i = 0; i < 200; ++i) {};
}

uint8_t prepare_callback(void *p)
{
    i2c_message *msg = (i2c_message *)p;
    // помечаем сообщение как отправленное
    msg->status = FINISHED_STATUS;
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

uint8_t basic_callback(void *p)
{
  prepare_callback(p);
  return 0;
}

void i2c_buffer_init(i2c_buffer *p)
{
  p->buf_head = 0;
  p->buf_tail = 0;
  p->count = 0;
}

uint8_t i2c_release_msg(i2c_buffer *p)
{
  if (p->count == 0)
  {
    //there is no message!
    return 1;
  }
  //if (p->buf_head == p->buf_tail)
  //{
    //there is no message!
  //  return 1;
  //}
  
  p->count--;
  p->buf_head++;
  if (p->buf_head >= I2C_BUFF_LEN)
  {
    p->buf_head = 0;
  }
  
  return 0;
}

//getting current message
uint8_t i2c_buffer_get(i2c_buffer *p, i2c_message **to)
{
  if (p->count == 0)
  {
    //buffer is empty
    return 1;
  }

  *to = &(p->message_buf[p->buf_head]);
  
  if ((*to)->status == FINISHED_STATUS)
  {
    //Todo: handle. Releasing message?
    //return 1;
  }
  return 0;
}


uint8_t i2c_buffer_add(i2c_buffer *p, i2c_message *from)
{
  if (from->status != IDLE_STATUS)
  {
    //message not new?
    //return 1;
  }
  
  if (p->count >= I2C_BUFF_LEN)
  {
    //buffer is full
    return 1;
  }
  
  i2c_message *dest = &(p->message_buf[p->buf_tail]);
  
  dest->device_addr = from->device_addr;
  dest->len = from->len;
  dest->reg_addr = from->reg_addr;
  dest->status = from->status;
  dest->direction = from->direction;
  dest->buf = from->buf;
  dest->f_callback = from->f_callback;
  
  p->count++;
  if ((p->buf_tail + 1) >= I2C_BUFF_LEN)
  //if (p->buf_tail >= I2C_BUFF_LEN)
  {
    p->buf_tail = 0;
  }
  else
  {
    p->buf_tail++;
  }
  
  if(p->buf_tail == p->buf_head)
  {
    uint8_t debug2 = 0;
    //buffer is full
  }
  
  return 0;
}

uint8_t read_reg_callback(void* p)
{
    i2c_message *msg = (i2c_message *)p;
    // помечаем сообщение как отправленное  
    //msg->status = FINISHED_STATUS;
    i2c_release_msg(&i2c_message_buffer);
    I2C_GenerateSTART(I2C1, ENABLE); // continue to read answer from IMU
    return 0;
}

uint8_t i2c_loop()
{
  i2c_message *msg;
  // Отправка и проверка статуса сообщений здесь
  uint8_t status = i2c_buffer_get(&i2c_message_buffer, &msg); 
  if (status)
  {
    return 1;
    // Todo: Call the handler here
  }
  //if message transmission not started
  if (msg->status == IDLE_STATUS)
  {
     uint8_t i2c_err_flag = I2C_GetFlagStatus(I2C1, I2C_FLAG_BERR | I2C_FLAG_RXNE | I2C_FLAG_BTF | I2C_FLAG_TXE);
//     uint8_t berr_flag = I2C_GetFlagStatus(I2C1, I2C_FLAG_BERR);
//     uint8_t rxne_flag = I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE);
//     uint8_t btf_flag = I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF);
//     uint8_t txe_flag = I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE);
     // Если шина свободна, поставить на отправку последнее сообщение из очереди
     if(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
     {
       if(i2c_err_flag)
       {
      // Генерирование СТАРТ-условия для передачи
          uint8_t debug = 0;
       }
       // Генерирование СТАРТ-условия для передачи
       I2C_GenerateSTART(I2C1, ENABLE);
     }
  }
  //Если сообщение было обработано в callback-функции
  if (msg->status == FINISHED_STATUS)
  {
     // удаляем сообщение из очереди
     i2c_release_msg(&i2c_message_buffer);
  }
  return 0;
}


int32_t i2c_read_reg(uint8_t device_addr, uint8_t reg, uint8_t *buf, uint16_t len, uint8_t (*finish_cb)(void* ))
{
  if(I2C_BUFF_LEN - i2c_message_buffer.count < 2)
  {
    return 1;
  }
   i2c_message msg;
   msg.device_addr = device_addr;
   msg.buf = 0; //not needed for writing 
   msg.direction = I2C_Direction_Transmitter;
   msg.len = 0; //because we send only one register that we want to read
   msg.reg_addr = reg;
   msg.status = IDLE_STATUS;
   msg.f_callback = read_reg_callback;
   
   uint8_t status = i2c_buffer_add(&i2c_message_buffer, &msg);
   if (status) return status;
   
   msg.device_addr = device_addr;
   msg.buf = buf;
   msg.direction = I2C_Direction_Receiver;
   msg.len = len;
   msg.reg_addr = 0;
   msg.status = START_STATUS;
   msg.f_callback = finish_cb;
   status = i2c_buffer_add(&i2c_message_buffer, &msg);
   if (status) return status;
   return 0;   
}


// MAK
int32_t i2c_write_reg(uint8_t device_addr, uint8_t reg, uint8_t *buf, uint16_t len, uint8_t (*f_cb)(void* ))
{
   i2c_message msg;
   msg.device_addr = device_addr;
   msg.buf = buf;  
   msg.direction = I2C_Direction_Transmitter;
   msg.len = len; 
   msg.reg_addr = reg;
   msg.status = IDLE_STATUS;
   msg.f_callback = f_cb;
   uint8_t status = i2c_buffer_add(&i2c_message_buffer, &msg);
   return status;
  
}

void i2c_sda_recover()
{   
  
#ifdef DEBUG_PU
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    for (uint8_t i = 0; i < 30; ++i)
    {
       if(i % 2) 
       {
         GPIO_SetBits(GPIOB, GPIO_Pin_6);
       }
       else
       {
         GPIO_ResetBits(GPIOB, GPIO_Pin_6);
       }
       gpio_i2c_delay();
    }
#endif

#ifdef DEBUG_STM3240G
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    for (uint8_t i = 0; i < 30; ++i)
    {
       if(i % 2) 
       {
         GPIO_SetBits(GPIOB, GPIO_Pin_8);
       }
       else
       {
         GPIO_ResetBits(GPIOB, GPIO_Pin_8);
       }
       gpio_i2c_delay();
    }
#endif
}

void i2c_init()
{
    I2C_DeInit(I2C1);
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Configure I2C_EE pins: SCL and SDA */
#ifdef DEBUG_PU
    
    i2c_sda_recover();

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
    
#endif
    
#ifdef DEBUG_STM3240G
    
    i2c_sda_recover();
    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
#endif
      
    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x38;

    
#ifdef I2C_ACK_ENABLE
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
#else
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;    
#endif
    ////
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;

    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the I2C event priority */
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C1, ENABLE);
}

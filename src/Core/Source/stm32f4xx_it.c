/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    16-jule-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include  "stm32f4xx_it.h"
#include  "stm32f40x_exti.h"
#include  "stm32f40x_spi.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

extern i2c_buffer i2c_message_buffer;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   //SysRstCode=NMIFault;  // code restart
   //NVIC_SystemReset();   // рестарт по зависанию фоновой ветви        
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
   //SysRstCode=HardFault;      // code restart
   //NVIC_SystemReset();   // рестарт по зависанию фоновой ветви        
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
   // SysRstCode=MemManage;      // code restart
   //NVIC_SystemReset();   // рестарт по зависанию фоновой ветви        
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
   //SysRstCode=BusFault;  // code restart
   //NVIC_SystemReset();   // рестарт по зависанию фоновой ветви        
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
   //SysRstCode=UsageFault;  // code restart
   //NVIC_SystemReset();   // рестарт по зависанию фоновой ветви        
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  proc_timer0();
};//

/******************************************************************************/
/*                 STM32Fxxx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32fxxx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */

#ifdef IMU_USE_INTERRUPTS
void EXTI4_IRQHandler(void)
{
  imu_IRQ();
}

void EXTI15_10_IRQHandler(void)
{
  imu_IRQ();
}
#endif

void EXTI0_IRQHandler(void)
{

}

void  SPI1_IRQHandler(void)   //void SPIx_IRQHANDLER(void)
{

}

void  DMA1_Stream0_IRQHandler(void) //; DMA1 Stream 0
{

};//

//----------------------------------------------------------------------
/**
  * @brief  This function handles DMA1_Stream4_IRQHandler.
  * Обработчик прерывания от ДМА1 поток4, процесса воспроизведения через
  * внешний конвертор
  * @param  None
  * @retval None
  */


//----------------------------------------------------------------------
/**
  * @brief  This function handles TIM5_IRQn global interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
  
}

/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  //USBD_OTG_ISR_Handler (&USB_OTG_dev);
}


void I2C1_EV_IRQHandler(void)
{
  //Todo: Check ALL INTERRUPTS
  uint32_t event = I2C_GetLastEvent(I2C1);
  uint8_t status = 0;
  
  i2c_message *msg;
  status = i2c_buffer_get(&i2c_message_buffer, &msg);
  if (status)
  {
    
    if(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE))
    {
      //dummy read
      I2C_ReceiveData(I2C1);
    }
    // Todo: clear interrupt before returning
    return;
  }
  
  // если получили на обработку уже завершенное сообщение
  if(msg->status == FINISHED_STATUS)
  {

  }
  
  //START_STATUS
  if((event & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT)
  {
    //START PASSED
    if (msg->status != IDLE_STATUS && msg->status != START_STATUS)
    {
      return;
    }
    msg->status = START_STATUS;
    //Discuss: do we have to use this in callbacks?
    I2C_Send7bitAddress(I2C1, msg->device_addr, msg->direction);
    return;
  }
  
////////////////////////////
// DONT CHANGE THE CALLING FUNCTION ORDER
////////////////////////////
  //TRANSMITTER ADDRESSED
  if((event & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
     == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
  {
    if (msg->status != START_STATUS)
    {
      uint8_t debug = 0;
      //wrong status
      return;
    }
    msg->status = ADDRESSED_STATUS;
    //send register byte
    I2C_SendData(I2C1, msg->reg_addr);
    return;
  }
  
  //RECEIVER ADDRESSED
  if((event & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) 
     == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
  {
    if (msg->status != START_STATUS)
    {
      //wrong status
      return;
    }
    msg->status = ADDRESSED_STATUS;
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return;
  }
//////////////////////////////
  
  //RECEIVED BYTE
  if((event & I2C_EVENT_MASTER_BYTE_RECEIVED) 
     == I2C_EVENT_MASTER_BYTE_RECEIVED)
  {
    if (msg->status != ADDRESSED_STATUS && msg->status != RECEIVING_STATUS)
    {
      //dumb receive
      I2C_ReceiveData(I2C1);
      //wrong status
      return;
    }
    
    msg->status = RECEIVING_STATUS;
    if (msg->buf != 0)
    {
      msg->buf[0] = I2C_ReceiveData(I2C1);
      msg->buf++;      
    }
    msg->len--;
    
    //if (msg->len <= 2)
    //{
    //  I2C_AcknowledgeConfig(I2C1, DISABLE);
    //}
    
    if (msg->len == 0)
    {
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      if(msg->f_callback != 0)
      { 
        msg->f_callback(msg);
      }
      else
      {
        msg->status = FINISHED_STATUS; // ЕСЛИ НЕТ CALLBACK функции - закрываем сообщение
      }
      return;
    }
    return;
  }
  
  if((event & I2C_EVENT_MASTER_BYTE_TRANSMITTED) 
     == I2C_EVENT_MASTER_BYTE_TRANSMITTED)
  {
    
    if (msg->status != ADDRESSED_STATUS && msg->status != TRANSMITTING_STATUS)
    {
      return;
    }
    
    msg->status = TRANSMITTING_STATUS;
    if(msg->len == 0)
    {
      if(msg->f_callback != 0) 
      {
        msg->f_callback(msg);
      }
      else
      {
        msg->status = FINISHED_STATUS; // ЕСЛИ НЕТ CALLBACK функции - закрываем сообщение
      }
      return;
    }
    
    if (msg->buf != 0 )
    {
      I2C_SendData(I2C1, msg->buf[0]);
      msg->buf++;      
    }
    msg->len--;
    return;
  }
  
  // Handle other events
  if(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE))
  {
    //dummy read
    I2C_ReceiveData(I2C1);
  }
  return;
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

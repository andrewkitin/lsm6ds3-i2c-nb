/**
  ******************************************************************************
  * @file    main.c 
  * @author  LKDS Application Team
  * @version V1.1.0
  * @date    21.06.2012
  * @brief   Main program body for Codec G723-1.
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdlib.h"
#include "lsm6ds3_test.h"
#include "i2c_nb.h"  
#include "bmp280_i2c.h"
//extern i2c_buffer buffer;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


    
uint8_t TIM0_Config()
{
    if (SysTick_Config(SystemCoreClock / 250)) // one tick per 4ms
    {
        /* Capture error */
        while (1);
    }
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 3));
}



/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main()
{  
  
  // boot
  
  // timer0
  TIM0_Config();
  //
  i2c_init();
  i2c_buffer_init(&i2c_message_buffer);

#ifdef USE_LSM6DS3
  IMU_init();
  IMU_setup();
#endif // USE_LSM6DS3
  
#ifdef USE_BMP280
  bmp_init();
  bmp_setup();
#endif // USE_BMP280
  
  while (1)
  {
    
    i2c_loop();
#ifdef FIFO_TEST
    if(imu_fifo_mode_on)
    {
      imu_fifo_read();
      imu_fifo_get_length();
    }
#endif //FIFO_TEST
    
#ifdef LEDS_TEST
    if(imu_accel_leds_test)
    {
      imu_accel_test_led();
    }
#endif // LEDS_TEST
    
    imu_accel_loop();
    
#ifdef USE_BMP280
    bmp_loop();
#endif // USE_BMP280
  }
}
/**
  ******************************************************************************
  * @file    stm32f40x_conf.h  
  * @author  MCD Application Team
  * @version V0.0.2
  * @date    25-06-2012
  * @brief   Library configuration file.
  ******************************************************************************
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F40x_CONF_H
#define __STM32F40x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "stm32f40x_adc.h"
#include "stm32f40x_can.h"
//#include "stm32f40x_crc.h"
//#include "stm32f40x_cryp.h"
#include "stm32f40x_dac.h"
//#include "stm32f40x_dbgmcu.h"
//#include "stm32f40x_dcmi.h"
#include "stm32f40x_dma.h"
#include "stm32f40x_exti.h"
#include "stm32f40x_flash.h"
//#include "stm32f40x_fsmc.h"
//#include "stm32f40x_hash.h"
#include "stm32f40x_gpio.h"
#include "stm32f40x_i2c.h"
#include "stm32f40x_iwdg.h"
//#include "stm32f40x_pwr.h"
#include "stm32f40x_rcc.h"
//#include "stm32f40x_rng.h"
//#include "stm32f40x_rtc.h"
//#include "stm32f40x_sdio.h"
#include "stm32f40x_spi.h"
#include "stm32f40x_syscfg.h"
#include "stm32f40x_tim.h"
#include "stm32f40x_usart.h"
//#include "stm32f40x_wwdg.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define 
   should be set to the value of the external clock source, else, if no external 
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */


/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F40x_CONF_H */

/******************* (C) COPYRIGHT 2012 LKDS *****END OF FILE****/

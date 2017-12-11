/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
    #include "arm_math.h"
    #include "defines.h"
    #include "ad7779.h"
    #include "communication.h"
    #include "crc16.h"
    #include "dsp.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
    ad7779_dev          ad7779device;
    uint8_t i;
    int32_t ad7779_setup_result = 0;
    readout_packet_t packet;
    uint32_t odr_time_us;
    uint8_t drdy_int = 0;
    uint16_t ampli[8] = {0};
    
    //dsp
    float32_t dsp_res1[ELECTRODESNUMBER];
    float32_t dsp_res2[ELECTRODESNUMBER];
    
    extern uint32_t redled_counter;
    extern uint32_t greenled_counter;
    extern DMA_HandleTypeDef hdma_adc1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ampli[0], 8);
  
  AD7779_VIBRO_DISABLE;
  RED_LED_DIS;
  GREEN_LED_DIS;
  BLUE_LED_DIS;
  
  /* На время инициализации АЦП включаем красный светодиод. */
  RED_LED_EN;
  AD7779_PWR_ENABLE;
  HAL_Delay(1000);
  AD7779_RESET_SET;
  HAL_Delay(100);
  AD7779_RESET_RESET;
  HAL_Delay(100);
  AD7779_RESET_SET;
  HAL_Delay(100);
 
  
  // Device Settings
ad7779device.spi_dev = AD7779_SPI;
ad7779device.ctrl_mode = AD7779_SPI_CTRL;
ad7779device.spi_crc_en = AD7779_DISABLE;
for (i = AD7779_CH0; i <= AD7779_CH7; i++) ad7779device.state[i] = AD7779_ENABLE;
for (i = AD7779_CH0; i <= AD7779_CH7; i++) ad7779device.gain[i] = AD7779_GAIN_1;

if (ODR == 1000)
{
  //Для прореживания в 2000 раз. Данные на выходе с частотой 1000 Гц.
  ad7779device.dec_rate_int = 0x07D0U;
} else
if (ODR == 500)
{
  //Для прореживания в 4000 раз. Данные на выходе с частотой 500 Гц.
  ad7779device.dec_rate_int = 0x0FA0U;
}
ad7779device.dec_rate_dec = 0;
ad7779device.ref_type = AD7779_INT_REF;
ad7779device.dclk_div = AD7779_DCLK_DIV_1;

/* Инициализация основных переменных */
odr_time_us = ( uint32_t ) ( 1000000.0f / ODR );
packet.prefix[ 0 ] = 'B';
packet.prefix[ 1 ] = 'L';
packet.length  = sizeof(payload_t);
packet.crc = 0;
packet.payload.cmd = 'M';

for (i = AD7779_REG_CH_CONFIG(0); i <= AD7779_REG_SRC_UPDATE; i++)
  {
    ad7779_spi_int_reg_read(&ad7779device, i, &ad7779device.cached_reg_val[i]);
  }
  
 /* Если инициализация прошла успешно выключаем красный светодиод.
    Включаем синий.
    Дергаем вибро 2 раза.
    Моргаем синим.
    Если сбой инициализации - красный остается гореть.
    Дальнейшее выполнение программы прекращается */
  ad7779_setup_result = ad7779_setup(&ad7779device);
  if (ad7779_setup_result == 0) RED_LED_DIS;
  else
  {
    for (;;);
  }

 AD7779_VIBRO_ENABLE;
 BLUE_LED_TOG;
 HAL_Delay(500);
 AD7779_VIBRO_DISABLE;
 BLUE_LED_TOG;
 HAL_Delay(500);
 AD7779_VIBRO_ENABLE;
 BLUE_LED_TOG;
 HAL_Delay(500);
 AD7779_VIBRO_DISABLE;
 BLUE_LED_DIS;
  
  HAL_Delay(100);
  ad7779_spi_int_reg_write(&ad7779device, AD7779_REG_GENERAL_USER_CONFIG_3, 0x10);
  HAL_Delay(100);
  //dsp
  dsp_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
     if (drdy_int)
  {
    /* Если данные от АЦП идут, горит зеленый светодиод.
    Если данные не идут зеленый светодиод выключен */
    GREEN_LED_EN;
    greenled_counter = 80;
    adc_read_sd(&ad7779device);
    //scale_redout(&ad7779device, &packet.payload.readout[0]);
    scale_redout(&ad7779device, dsp_res1);
    //dsp
    dsp_pocess();
    uint16_t adcscale = adc_scale();
    for (i=0; i<ELECTRODESNUMBER; i++)
    {
      packet.payload.readout[i] = adcscale * dsp_res2[i];
    }
    
    // Prepare Packet
    packet.payload.counter += odr_time_us;
    packet.crc = buildCRC(&packet);

    HAL_UART_Transmit( &AD7779_UART , ( uint8_t* )&packet , sizeof( packet )  , 2 );
    drdy_int = 0;
  }
    if (greenled_counter <= 0) GREEN_LED_DIS;
    
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

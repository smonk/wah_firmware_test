/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2s.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "system_defs.h"
#include "wm8731.h"
#include "hardFaultHandler.h"
#include "tremolo_one.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

//how many samples per data frame per?
// #define AUDIO_BLOCK_SIZE 256
// #define ADC_POT_BUFFER_SIZE 8
// #define SAMPLING_RATE 32000 
/*we need a few buffers */
int16_t codec_rx_buffer_a[AUDIO_BLOCK_SIZE*2];
int16_t codec_tx_buffer_a[AUDIO_BLOCK_SIZE*2];

//this is the buffer the ADC DMA will write values into
//once it is full you can average them in the interrupt and use that as the pot value
uint16_t wah_pot_buffer[ADC_POT_BUFFER_SIZE];

/*variables for audio dma buffer*/
int32_t block_tick = 0;
int32_t block_half= 0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
//the patch pointer

int32_t (* patch)( float *, float *, int32_t);

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
  //clear the DMA buffers
  uint32_t j = 0;
  for(j = 0; j < AUDIO_BLOCK_SIZE*2; j++){
    codec_tx_buffer_a[j] = 0;
    codec_rx_buffer_a[j] = 0;
  }

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();  //
  MX_ADC3_Init(); //wah pot input
  MX_DAC_Init();  //cv outs
  MX_I2S3_Init(); //i2s codec
  MX_SPI2_Init(); //spi codec

  /* USER CODE BEGIN 2 */
  codec_init( & hspi2 );  
  HAL_SuspendTick(); 
  HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)codec_tx_buffer_a, (uint16_t *)codec_rx_buffer_a, AUDIO_BLOCK_SIZE * 2);

  float input_block[AUDIO_BLOCK_SIZE];
  float output_block[AUDIO_BLOCK_SIZE];
  uint32_t start_a, end_a = 0;
  uint32_t frame_number = 0;

  float amplitude = 0;
  float amplitude_delta = .1;
  float amplitude_max = 1;
  uint32_t lfo_t = 0;
  uint32_t lfo_T = 100;


  /* USER CODE END 2 */


  patch = tremolo_one;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    while(block_tick){};
    block_tick = 1;
    if(block_half){//use second half 
      start_a = AUDIO_BLOCK_SIZE;
      end_a = AUDIO_BLOCK_SIZE * 2;
      for(j= start_a; j < end_a; j++){
        input_block[j - AUDIO_BLOCK_SIZE] = (float)codec_rx_buffer_a[j] / 32768.0;
        codec_tx_buffer_a[j] = (int16_t)(output_block[j - AUDIO_BLOCK_SIZE] * 32768.0); 
      }
    }else{//use first half
      start_a = 0;
      end_a = AUDIO_BLOCK_SIZE;
      for(j= start_a; j < end_a; j++){
        input_block[j] = (float)codec_rx_buffer_a[j] / 32768.0;
        codec_tx_buffer_a[j] = (int16_t)(output_block[j] * 32768.0); 
      }
    }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    //simple loop through
    // for( j = 0; j < AUDIO_BLOCK_SIZE; j++){
    //   output_block[j] = input_block[j];
    // }

    patch(input_block,output_block, AUDIO_BLOCK_SIZE);

    frame_number++;

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
uint32_t nt = 0;
void HAL_I2S_TxCpltCallback ( I2S_HandleTypeDef * hi2s){
 nt++;

}
uint32_t nr = 0;
void HAL_I2S_RxCpltCallback ( I2S_HandleTypeDef * hi2s){ 
 nr++;
  block_tick = 0;
  block_half = 1;
}

uint32_t nth = 0;
void HAL_I2S_TxHalfCpltCallback ( I2S_HandleTypeDef * hi2s){  
  nth++;

} 

uint32_t nrh = 0;
void HAL_I2S_RxHalfCpltCallback ( I2S_HandleTypeDef * hi2s){
  nrh++;
  block_tick = 0;
  block_half = 0;
} 
/* USER CODE END 4 */

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

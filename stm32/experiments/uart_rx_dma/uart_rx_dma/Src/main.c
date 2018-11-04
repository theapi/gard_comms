
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32l0xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* DMA Timeout event UART fronm
 * https://github.com/akospasztor/stm32-dma-uart
 *
 * NB:
 *   main.h defines the DMA_Event_t struct
 *   usart.c has UART2 IDLE Interrupt Configuration in HAL_UART_MspInit()
 *   usart.c provides DMA_Event_t dma_uart_rx
 *   usat.h exports dma_uart_rx which stm32l0xx_it.c uses with extern DMA_Event_t dma_uart_rx;
 *   stm32l0xx_it.c has DMA timer in SysTick_Handler()
 *   stm32l0xx_it.c has UART IDLE Interrupt in USART2_IRQHandler()
 *
 *   uart is confirgured as dma circular buffer.
*/


uint8_t dma_rx_buf[DMA_BUF_SIZE];       /* Circular buffer for DMA */
uint8_t data[DMA_BUF_SIZE] = {'\0'};    /* Data buffer that contains newly received data */

__IO ITStatus RxReady = RESET;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);


  if (HAL_UART_Receive_DMA(&huart2, (uint8_t*)dma_rx_buf, DMA_BUF_SIZE) != HAL_OK) {
  		     Error_Handler();
  		  }
  		  /* Disable Half Transfer Interrupt */
  		        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  // RX buffer has been processed, listen again for more.
	  if (RxReady == RESET) {

	  }
	  if (RxReady == SET) {
//		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		  // Blocking.
//		  HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), 200);
//		  RxReady = RESET;
	  }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/** DMA Rx Complete AND DMA Rx Timeout function
 * Timeout event: generated after UART IDLE IT + DMA Timeout value
 * Scenarios:
 *  - Timeout event when previous event was DMA Rx Complete --> new data is from buffer beginning till (MAX-currentCNDTR)
 *  - Timeout event when previous event was Timeout event   --> buffer contains old data, new data is in the "middle": from (MAX-previousCNDTR) till (MAX-currentCNDTR)
 *  - DMA Rx Complete event when previous event was DMA Rx Complete --> entire buffer holds new data
 *  - DMA Rx Complete event when previous event was Timeout event   --> buffer entirely filled but contains old data, new data is from (MAX-previousCNDTR) till MAX
 * Remarks:
 *  - If there is no following data after DMA Rx Complete, the generated IDLE Timeout has to be ignored!
 *  - When buffer overflow occurs, the following has to be performed in order not to lose data:
 *      (1): DMA Rx Complete event occurs, process first part of new data till buffer MAX.
 *      (2): In this case, the currentCNDTR is already decreased because of overflow.
 *              However, previousCNDTR has to be set to MAX in order to signal for upcoming Timeout event that new data has to be processed from buffer beginning.
 *      (3): When many overflows occur, simply process DMA Rx Complete events (process entire DMA buffer) until Timeout event occurs.
 *      (4): When there is no more overflow, Timeout event occurs, process last part of data from buffer beginning till currentCNDTR.
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t i, pos, start, length;
    uint16_t currCNDTR = __HAL_DMA_GET_COUNTER(huart->hdmarx);

    /* Ignore IDLE Timeout when the received characters exactly filled up the DMA buffer and DMA Rx Complete IT is generated,
       but there is no new character during timeout */
    if(dma_uart_rx.flag && currCNDTR == DMA_BUF_SIZE)
    {
        dma_uart_rx.flag = 0;
        return;
    }

    /* Determine start position in DMA buffer based on previous CNDTR value */
    start = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (DMA_BUF_SIZE - dma_uart_rx.prevCNDTR) : 0;

    if(dma_uart_rx.flag)    /* Timeout event */
    {
        /* Determine new data length based on previous DMA_CNDTR value:
         *  If previous CNDTR is less than DMA buffer size: there is old data in DMA buffer (from previous timeout) that has to be ignored.
         *  If CNDTR == DMA buffer size: entire buffer content is new and has to be processed.
        */
        length = (dma_uart_rx.prevCNDTR < DMA_BUF_SIZE) ? (dma_uart_rx.prevCNDTR - currCNDTR) : (DMA_BUF_SIZE - currCNDTR);
        dma_uart_rx.prevCNDTR = currCNDTR;
        dma_uart_rx.flag = 0;
    }
    else                /* DMA Rx Complete event */
    {
        length = DMA_BUF_SIZE - start;
        dma_uart_rx.prevCNDTR = DMA_BUF_SIZE;
    }

    /* Copy and Process new data */
    for(i=0,pos=start; i<length; ++i,++pos)
    {
        data[i] = dma_rx_buf[pos];
    }

    /* Data ready for using */
    RxReady = SET;

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    		  // Blocking.
    		  HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), 200);
    		  RxReady = RESET;

}

/* Error callback */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Error_Handler();
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

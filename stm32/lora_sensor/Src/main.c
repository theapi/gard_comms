/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdio.h"

#include "rfm95.h"
#include "payload_solar.h"
#include "ads1015.h"
#include "battery.h"
#include "temperature.h"
#include "light.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TXBUFFERSIZE 128
#define RXBUFFERSIZE 1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t dio0_action = 0;
MAIN_StateTypeDef state;

/* Buffer used for debug UART */
char tx_buffer[TXBUFFERSIZE];
char aRxBuffer[RXBUFFERSIZE];
volatile uint8_t txBuffer[TXBUFFERSIZE];
volatile uint8_t txIndex = 0;
//__IO ITStatus txReady = RESET;
volatile RX_StateTypeDef rxStatus = RX_STATE_EMPTY;
volatile PAYLOAD_Solar payload;
uint8_t payload_buff[PAYLOAD_Solar_SIZE];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);

    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower();
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);


    RFM95_init(&hspi2);



    payload.MessageType = 55;
    payload.DeviceId = 2;
    payload.MessageId = 0;
    payload.Flags = 0;
    payload.Light = 0; // Not got a light sensor attached to this device.

    // Start in sensing mode.
    state = MAIN_STATE_SENSE;
    //state = MAIN_STATE_SLEEP; //TMP sleep until UART payload

    /* Listen to the UART */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /* Do some work */
        if (state == MAIN_STATE_SENSE) {
            HAL_ADC_Start(&hadc);
            // Junk readings as the adc just turned on.
            payload.Temperature = TEMPERATURE_external();
            payload.CpuTemperature = TEMPERATURE_cpu();

            HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);

            payload.MessageId++;
            payload.VCC = BATTERY_vcc();
            payload.ChargeMv = BATTERY_ChargeMv();
            payload.ChargeMa = BATTERY_ChargeMa();


            // No external temperature sensor attached.
            // BUT TEMPERATURE_external() must still be called :(
            payload.Temperature = TEMPERATURE_external();
            //payload.Temperature = 0;

            payload.CpuTemperature = TEMPERATURE_cpu();
            HAL_ADC_Stop(&hadc);

            // Send data to debug UART.
			int tx_len = snprintf(
			  tx_buffer,
			  TXBUFFERSIZE,
			  "msg_id:%d, device_id:%u, flags:%X, vcc:%u, ma:%d, cpuC:%d\n",
			  payload.MessageId,
			  payload.DeviceId,
			  payload.Flags,
			  payload.VCC,
			  payload.ChargeMa,
			  payload.CpuTemperature
			);
			// Blocking UART.
			HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, tx_len, 500);

            PAYLOAD_Solar_serialize(payload, payload_buff);
            RFM95_send(&hspi2, payload_buff, PAYLOAD_Solar_SIZE);

            state = MAIN_STATE_TX;
            /* Reset the flags as they must be set by the UART */
            payload.Flags = 0;
        }

        /* Do nothing while the transmission is in progress */
        else if (state == MAIN_STATE_TX) {
            if (dio0_action == 1) {
                RFM95_setMode(&hspi2, RFM95_MODE_SLEEP);
                state = MAIN_STATE_SLEEP;
            }

            //TMP while interrupts are investigated
            //HAL_Delay(30);
            //state = MAIN_STATE_SLEEP;
        }

        /* Now that all the work is done, sleep until its time to do it all again */
        else if (state == MAIN_STATE_SLEEP) {
            HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);




//            // RX buffer has been processed, listen again for more.
//			  if (rxStatus == RX_STATE_EMPTY || rxStatus == RX_STATE_INCOMING) {
//				  payload.Flags = 0;
//				  // Non blocking.
//				  HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
//			  } else if (rxStatus == RX_STATE_FULL) {
//				  // Got a UART message
//				  payload.Flags = txBuffer[0];
//				  //state = MAIN_STATE_SENSE;
//			  }

              //TMP
              HAL_Delay(17000);

//            /* Turn off the pin interrupts */
//            HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
//
//            HAL_SuspendTick();
//
//            /* Enter Stop Mode */
//            HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60,
//            RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
//            HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//            HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
//            HAL_ResumeTick();
//
//            /* Turn on the pin interrupts */
//            HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

              state = MAIN_STATE_SENSE;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* RTC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

/**
 * Callback for RTC_IRQHandler()
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
    // Wake up
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t c = aRxBuffer[0];

	/* Wait for a new payload which starts with a tab. */
	if (rxStatus == RX_STATE_EMPTY) {
		if (c == 0x09) {
			rxStatus = RX_STATE_INCOMING;
			// Start at the beginning of the buffer.
			txIndex = 0;
			/* Don't store the tab */
		}
	}
	/* Do nothing until a tab is seen */
	else if (rxStatus == RX_STATE_INCOMING) {
		// Prevent buffer overflow.
		if (txIndex >= TXBUFFERSIZE) {
			txIndex = 0;
		}

		// Add the character to the tx buffer.
		txBuffer[txIndex++] = c;

		// Keep adding to the tx buffer until the new line character "\n".
		if (c == 0x0A) {
			// Got all the payload.
			rxStatus = RX_STATE_EMPTY;
			payload.Flags = txBuffer[0];
			/* Force a send */
			state = MAIN_STATE_SENSE; // Currently not instant due to the HAL_DELAY()
		}
	}

	HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

/**
 * Callback for HAL_GPIO_EXTI_IRQHandler() in EXTI4_15_IRQHandler().
 *
 * Handles the interrupts which occur on button press.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == DIO0_Pin) {
        dio0_action = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, LED_Pin);
        HAL_Delay(100);
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

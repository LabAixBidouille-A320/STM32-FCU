
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
  * Cet exemple teste l'affichage de la partie gauche du FCU
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
//#define    OPCODEW       (0b01000000)
//#define    OPCODER       (0b01000001)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t data_receive;
char Rx_indx, Rx_Buffer[100], buffer[100];
uint8_t len = 0, Transfer_cplt, Rx_data[2], spidata[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Afficher sur le port série
void log(char* str)
{
	HAL_UART_Transmit(&huart2, str, strlen(str), 1000);
}

// Recevoir une donnée du port série par interruption
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i;

	if(huart->Instance == USART2)
	{
		if(Rx_indx==0) {for(i=0; i<100;i++) Rx_Buffer[i]=0;}

		if(Rx_data[0]!=10)
		{
			Rx_Buffer[Rx_indx++]=Rx_data[0];
		}
		else
		{
			Rx_indx = 0;
			Transfer_cplt = 1;
		}

		HAL_UART_Receive_IT(&huart2, Rx_data, 1);
	}
}
/*
void spiTransfer(GPIO_TypeDef * port, int pin, int addr, uint8_t opcode, uint8_t data) {
    //Create an array with the data to shift out
    int offset=addr*2;
    int maxbytes=1*2;

    for(int i=0;i<maxbytes;i++)
        spidata[i]=0;
    //put our device data into the array
    spidata[offset+1]=opcode;
    spidata[offset]=data;
    //enable the line
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    //Now shift out the data
    for(int i=maxbytes;i>0;i--)
    {
    	uint8_t j ;
    	uint8_t transmit;

		for ( j=0 ; j < 8 ; j++ )
		{
			transmit = spidata[i-1] & (1 << (7 - j));
			HAL_SPI_Transmit(&hspi3, &transmit , 1, 1000);

			/* HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, spidata[i-1] & (1 << (7 - j)));
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    	}
    }
    //latch the data onto the display
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

} */

// Transmettre une donnée au FCU en SPI
void FCU_Transmit(GPIO_TypeDef * CS_port, int CS_pin, uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
}

/* void FCU_TransmitSW(uint8_t addr, uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, OPCODEW | (addr << 1) , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &data , 1, 1000);
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_SET);
}

uint8_t FCU_ReceiveSW(uint8_t addr, uint8_t reg, uint8_t data)
{
	uint8_t value = 0;
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, OPCODER | (addr <<  1) , 1, 1000);
	HAL_SPI_Transmit(&hspi3, &reg , 1, 1000);
	HAL_SPI_TransmitReceive(&hspi3, &data , &value, 1, 1000);
	HAL_GPIO_WritePin(NSS_Switch_GPIO_Port, NSS_Switch_Pin, GPIO_PIN_SET);
	return value;
} */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint8_t ad = 0;
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
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 0x0C, 0x01); // No shutdown
  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 0x0B, 0x05); // Scan Limit 0-5
  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 0x09, 0x0F); // Decode Mode, digits 3-0

  for(uint8_t i = 1; i < 5; i++)
  {
	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, i, 0x88); // Allumer les digits
  }
  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 5, 0xFF); //Allumer les leds
  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 6, 0xFF); // Allumer les voyants

  for(uint8_t i = 0; i < 15; i++)
  {
	  HAL_Delay(100);
	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 0x0A, i); // Changer l'intensité
  }
  for(uint8_t i = 15; i > 0; i--)
  {
	  HAL_Delay(100);
  	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 0x0A, i); // Changer l'intensité
  }
  for(uint8_t i = 0; i < 15; i++)
  {
	  HAL_Delay(100);
	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 0x0A, i); // Changer l'intensité
  }

  for(uint8_t i = 1; i < 5; i++)
  {
	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, i, 0x0F); // Eteindre les digits
  }
  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 5, 0x00); //Eteindre les leds
  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 6, 0x00); // Eteindre les voyants

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 for (uint8_t i = 1; i < 5; i++)
		{
		  for (uint8_t j = 0; j < 16; j++)
		  {
			  HAL_Delay(200);
			  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, i, j); // Afficher la valeur j sur le digit numero i
		  }
		}
	  for (uint8_t i = 1; i < 65; i*=2)
	  {
		  HAL_Delay(200);
		  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 5, i); // Transmet la valeur i sur les leds
	  }
	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 5, 0); // Eteindre les leds

	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 6, 3); // Allumer le voyant gauche
	  HAL_Delay(200);
	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 6, 96); // Allumer le voyant droit
	  HAL_Delay(200);
	  FCU_Transmit(NSS_AffG_GPIO_Port, NSS_AffG_Pin, 6, 0); // Eteindre les voyants

	  /*
	  FCU_TransmitSW(ad, 0x0A, 0x08);
	  FCU_TransmitSW(ad, 0x00, 0xFF);
	  FCU_TransmitSW(ad, 0x01, 0xFF);
	  FCU_TransmitSW(ad, 0x0C, 0xFF);
	  FCU_TransmitSW(ad, 0x0D, 0xFF);

	  data_receive = FCU_ReceiveSW(ad, 0x12, 0x00);

	  log( "\nGPIOA : ");
	  log(&data_receive);
	  log( "\n");

	  data_receive = FCU_ReceiveSW(ad, 0x13, 0x00);
	  log( "\nGPIOB : ");
	  log(&data_receive);
	  log( "\n");
	   */

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|NSS_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Switch_Pin|NSS_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_AffD_GPIO_Port, NSS_AffD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NSS_AffG_Pin|NSS_AffC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : POT_INT_Pin */
  GPIO_InitStruct.Pin = POT_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POT_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 NSS_Reset_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|NSS_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Switch_Pin NSS_BL_Pin */
  GPIO_InitStruct.Pin = NSS_Switch_Pin|NSS_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_AffD_Pin */
  GPIO_InitStruct.Pin = NSS_AffD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NSS_AffD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_AffG_Pin NSS_AffC_Pin */
  GPIO_InitStruct.Pin = NSS_AffG_Pin|NSS_AffC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

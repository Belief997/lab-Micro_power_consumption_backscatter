/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define AD9838_Control_Port  GPIOA
#define AD9838_RESET  GPIO_PIN_8		
#define AD9838_SDATA  GPIO_PIN_9 		
#define AD9838_SCLK   GPIO_PIN_10			
#define AD9838_FSYNC  GPIO_PIN_11   


#define AD9838_FSYNC_SET   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_FSYNC, GPIO_PIN_SET) 
#define AD9838_FSYNC_CLR   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_FSYNC, GPIO_PIN_RESET) 
#define AD9838_SCLK_SET   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SCLK, GPIO_PIN_SET) 
#define AD9838_SCLK_CLR   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SCLK, GPIO_PIN_RESET) 
#define AD9838_SDATA_SET   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SDATA, GPIO_PIN_SET) 
#define AD9838_SDATA_CLR   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_SDATA, GPIO_PIN_RESET) 
#define AD9838_RESET_SET   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_RESET, GPIO_PIN_SET) 
#define AD9838_RESET_CLR   HAL_GPIO_WritePin(AD9838_Control_Port ,AD9838_RESET, GPIO_PIN_RESET) 


void delay_n(int time)
{
	int i = 0;
	while(i++<20*time);
	
	
}


void AD9838_Write_16Bits(unsigned int data) 
{ 
    unsigned char i = 0 ; 

    AD9838_SCLK_SET ; 
    AD9838_FSYNC_CLR ; 

    for(i=0 ;i<16 ;i++)      
		{ 
        if(data & 0x8000) 
          AD9838_SDATA_SET ; 
        else 
          AD9838_SDATA_CLR ; 

        AD9838_SCLK_CLR ; 
        data <<= 1 ; 
        AD9838_SCLK_SET ; 
    } 
    AD9838_SDATA_SET ; 
    AD9838_FSYNC_SET ; 
} 

#define Triangle_Wave    0x2002 
#define Sine_Wave  0x2008 
#define Square_Wave 0x2028

void AD9838_Select_Wave(unsigned int initdata) 
{ 
    AD9838_FSYNC_SET; 
    AD9838_SCLK_SET; 
    AD9838_RESET_SET;  
    AD9838_RESET_CLR;
    AD9838_Write_16Bits(initdata); 
} 


void AD9838_Init() 
{
		AD9838_RESET_SET;
AD9838_SCLK_CLR ; 	

	AD9838_FSYNC_SET;	
		AD9838_SCLK_SET;
		AD9838_SDATA_SET;
		AD9838_RESET_CLR;

	
	
		AD9838_Write_16Bits(0x2100);
		AD9838_Write_16Bits(0x2038);
		AD9838_Write_16Bits(0XC000);
		AD9838_Write_16Bits(0x2100);
} 

#define AD9838_SYSTEM_COLCK     16000000UL //16MHzÎªAD9838Ö÷Ê±ÖÓ
#define FREQ_0      0 
#define FREQ_1      1 
void AD9838_Set_Freq(unsigned char freq_number, unsigned long freq) 
{
    unsigned long FREQREG = (unsigned long)(268435456.0/AD9838_SYSTEM_COLCK*freq); 
    unsigned int FREQREG_LSB_14BIT = (unsigned int)FREQREG; 
    unsigned int FREQREG_MSB_14BIT = (unsigned int)(FREQREG>>14); 
	
    if(freq_number == FREQ_0) 
    { 
        FREQREG_LSB_14BIT &= ~(1U<<15); 
        FREQREG_LSB_14BIT |= 1<<14; 
        FREQREG_MSB_14BIT &= ~(1U<<15); 
        FREQREG_MSB_14BIT |= 1<<14; 
    }
    else
    {
        FREQREG_LSB_14BIT &= ~(1<<14); 
        FREQREG_LSB_14BIT |= 1U<<15; 
        FREQREG_MSB_14BIT &= ~(1<<14); 
        FREQREG_MSB_14BIT |= 1U<<15; 
    } 
    AD9838_Write_16Bits(FREQREG_LSB_14BIT); 
    AD9838_Write_16Bits(FREQREG_MSB_14BIT);      
} 


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
  /* USER CODE BEGIN 2 */

	AD9838_Init() ;
	AD9838_Select_Wave(Square_Wave) ;
	AD9838_Set_Freq(FREQ_1, 4025000);
	AD9838_Set_Freq(FREQ_0, 3975000);


		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
				delay_n(100);
		AD9838_Write_16Bits(0x2038);
		delay_n(100);
		AD9838_Write_16Bits(0x2838);

		
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

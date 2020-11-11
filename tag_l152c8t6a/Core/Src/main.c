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
#include "string.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "radio.h"
#include "platform.h"
#include "sx1276-Hal.h"
#include "stm32f10x_type.h"
#include <stdlib.h>
#include "stm32l1xx_hal_gpio.h"
#include "sx1276-Fsk.h"

#define BUFFER_SIZE                                 18 // Define the payload size here
#define MAX_USER                                    5  
#define TIME_GAP                                    1000 //ms

#define RX_TIMMER_SYNC                              2 // byte position of sync time 
#define SENSOR_BYTE                                 7
#define SENSOR_TIMEOUT															500 //ms
extern volatile uint32_t TickCounter_NTZ;
extern volatile uint32_t TickCounter;
static uint16_t BufferSize = BUFFER_SIZE;			// RF buffer size
static uint8_t Buffer[BUFFER_SIZE];					// RF buffer
static uint8_t EnableMaster = true; 				// Master/Slave selection
tRadioDriver *Radio = NULL;
const uint8_t PingMsg[] = "1234";
const uint8_t PongMsg[] = "5678";   
uint8_t tx_cmd = 0; 

volatile uint32_t TxTimer;

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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//char txBuf1[18] = {'5','0','8','7','9',0xff,'5','0','8','7','9',0xff,'5','0','8','7','9',0xff};
char txBuf1[18] = {'4','3','3','0','0','0'};

char txBuf2[20] = "hello fuck world";
char rxBuf1[7] = "11111";
char rxBuf2[20];
char rcv_flag1 = 0;
char rcv_flag2 = 0;
volatile char read_over = 0;


#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	if(huart == &huart1){
		//HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuf1, SENSOR_BYTE);
		read_over = 1;
	}
//	else if(huart == &huart2){
//		HAL_UART_Transmit(&huart2, (uint8_t *)rxBuf2, strlen(txBuf2), 0xffff);
//		memset(rxBuf2,0, strlen(txBuf1));
//		HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuf2, 10);
//	}
}

/*
 * Manages the slave operation
 */
void OnSlave( void )
{
    uint8_t i;
		static uint32_t time_pos_sec,time_pos_ms = 0;    //send message on time_pos sec
    while(Radio->Process()==RF_RX_DONE){
        Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
				//HAL_UART_Transmit(&huart1, (uint8_t *)Buffer, strlen(Buffer), 0xffff);
				if(strcmp((const char *)Buffer,txBuf1)==0){ 
					//connection established
					toggle_led();
					TickCounter = Buffer[RX_TIMMER_SYNC] * TIME_GAP;
					time_pos_ms = time_pos_sec * TIME_GAP;
				}else{   
				//connection lost, retreat rand sec, range from (0,MAX_USER-1)
					srand(TickCounter_NTZ);
					time_pos_ms = TickCounter + rand()%MAX_USER*1000;
				}
				
				if(TickCounter-time_pos_ms >= MAX_USER*TIME_GAP){
					TickCounter = time_pos_ms;
					
					Buffer[0] = '5';
					Buffer[1] = '0';
					Buffer[2] = '9';
					Buffer[3] = '7';
					Buffer[4] = '9';					
					Buffer[5] = 0xfe;
					// We fill the buffer with numbers for the payload 
					Radio->SetTxPacket( Buffer, 6 );
					toggle_led();
				}
		}
		
    switch( Radio->Process( ) )
    {
    case RF_RX_DONE:
        Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
				HAL_UART_Transmit(&huart1, (uint8_t *)Buffer, strlen(Buffer), 0xffff);
        if( BufferSize > 0 )
        {
            if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
            {
                // Indicates on a LED that the received frame is a PING
                //LedToggle( LED_GREEN );

               // Send the reply to the PONG string
                Buffer[0] = '5';
                Buffer[1] = '6';
                Buffer[2] = '7';
                Buffer[3] = '8';
                // We fill the buffer with numbers for the payload 
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }

                Radio->SetTxPacket( Buffer, BufferSize );
            }
        }
        break;
    case RF_TX_DONE:
        // Indicates on a LED that we have sent a PONG
        //LedToggle( LED_RED );
        Radio->StartRx( );
        break;
    default:
        break;
    }
}


#endif

typedef enum
{
    STATE_BKSCT_IDLE = 0,
    STATE_BKSCT_BUSY = 1,


};

volatile u8 State_bksct = STATE_BKSCT_IDLE;


/*
 * Manages the master operation
 */

 


#define FSK_PREAMBLE (0XAA)
#define FSK_PREAMBLE_LEN (2)//(5)
#define FSK_SYNC_WORD (0X000000C1) // (0X00C194C1)
#define FSK_SYNC_LEN (1) //(3)
#define FSK_PAYPLOAD_LEN (6)
#define FSK_CRC_LEN (2)
#define FSK_FRAME_LEN (FSK_PREAMBLE_LEN + FSK_SYNC_LEN + 1 + FSK_PAYPLOAD_LEN + FSK_CRC_LEN) // 5() + 3 + 1 + 6 + 2
 
#define n2s16(x) (((x & 0xff00) >> 8) | ((x & 0x00ff) << 8))


 u8 fskLen = 0;
 static u8 fskBuff[2*FSK_FRAME_LEN] = {0};

 // crc
 u16 CRC_calc(u8 *buffer, u8 bufferLength)
 {
     u16 crc = 0x1D0F;
     u16 poly = 0x1021;
 
     u8 i;
     for(i = 0; i < bufferLength; i++)
     {
         u8 data = buffer[i];
         u8 j;
         for(j = 0; j < 8; j++)
         {
             if( (( (crc & 0x8000) >> 8 ) ^ (data & 0x80)) != 0 )
             {
                 crc <<= 1;
                 crc ^= poly;
             }
             else
             {
                 crc <<= 1;
             }
             data <<= 1;
         }
     }
     return (u16)(~crc);
 }
 
 //
 static u8 array_lfsr[] = {
 //      11111111 10000111 10111000 01011001
 //      10110111 10100001 11001100 00100100
 //      01010111 01011110 01001011 10011100
 //      00001110 11101001 11101010 01010000
 //      00101010 10111110
 
         0xff, 0x87, 0xb8, 0x59,
         0xb7, 0xa1, 0xcc, 0x24,
         0x57, 0x5e, 0x4b, 0x9c,
         0x0e, 0xe9, 0xea, 0x50,
         0x2a, 0xbe
 };
 
 // I/O buffer size should be same
 s8 user_whitening(u8 *bufferIn, u8 lenIn, u8 *bufferOut)
 {
     if(lenIn > sizeof(array_lfsr))
     {
         // data is oversized
         return -1;
     }
 
     u8 i = 0;
     for(i = 0; i < lenIn; i++)
     {
         bufferOut[i] = bufferIn[i] ^ array_lfsr[i];
     }
     return 0;
 }
 

#define FSK_CRC 1
#define FSK_WHITENING 0
 u8 user_fskFrame(u8 *fskBuff, u8 fskBufSize, u8 *data, u8 dataLen)
 {
     if(dataLen != FSK_PAYPLOAD_LEN || fskBufSize < FSK_FRAME_LEN)
     {
         // wrong dataLen
         return 0;
     }
 
     u8 *pdata = fskBuff;
 
     memset(pdata, FSK_PREAMBLE, FSK_PREAMBLE_LEN);
     pdata += FSK_PREAMBLE_LEN;
 
     u32 temp = FSK_SYNC_WORD;
     memcpy(pdata, &temp, FSK_SYNC_LEN);
     pdata += FSK_SYNC_LEN;

     // dataBuff[0] as raw data buff, set length and copy into data here
     u8 dataBuff[2][FSK_PAYPLOAD_LEN + 1 + FSK_CRC_LEN] = {FSK_PAYPLOAD_LEN};
     memcpy(&dataBuff[0][1], data, FSK_PAYPLOAD_LEN);

     // copy crc to dataBuff[0]
#if FSK_CRC == 1
     temp = n2s16(CRC_calc(dataBuff[0], (u8)(FSK_PAYPLOAD_LEN + 1)));
#else
    temp = 0;
#endif
     memcpy(&dataBuff[0][0]+FSK_PAYPLOAD_LEN + 1, &temp, FSK_CRC_LEN);

#if FSK_WHITENING == 1
    // whitening (len + data + crc) and copy to dataBuff[1]
    user_whitening(dataBuff[0], FSK_PAYPLOAD_LEN + 1 + FSK_CRC_LEN, dataBuff[1]);
    memcpy(pdata, dataBuff[1], FSK_PAYPLOAD_LEN + 1 + FSK_CRC_LEN);
#else
     memcpy(pdata, dataBuff[0], 1 + FSK_PAYPLOAD_LEN + FSK_CRC_LEN);
#endif
     pdata += FSK_PAYPLOAD_LEN + 1 + FSK_CRC_LEN;
 
     return (pdata - fskBuff);
 }
 
 
#if 0
void OnMaster( void )
{
    uint8_t i,sensor_is_timeout;
		uint32_t tickstart;
    while(1){
				if(TickCounter-TxTimer>=1000){
					TxTimer = TickCounter;
					//Without sensor.
					/*
					Radio->SetTxPacket( txBuf1, SENSOR_BYTE-1 );
					toggle_led();
					*/
					//Connect sensor
					
					tri_sensor();
//					while(read_over==0)
//						if(TickCounter-TxTimer>=SENSOR_TIMEOUT)
//							break;
					
					sensor_is_timeout = (TickCounter-TxTimer>=SENSOR_TIMEOUT);
					sensor_is_timeout = 1;
					
					read_over=0;
					HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuf1, SENSOR_BYTE);
					if(0==sensor_is_timeout){
						if(rxBuf1[0]!=0x00)
								Radio->SetTxPacket( rxBuf1, SENSOR_BYTE-1 );
						else
							Radio->SetTxPacket( rxBuf1+1, SENSOR_BYTE-1 );
						toggle_led();
					}
					else
					{
//						char timeout_buf[] = "_nope";
//						Radio->SetTxPacket( timeout_buf, SENSOR_BYTE-1 );
                        u8 data[] = {0XAA, 0X31, 0XAA, 0X32, 0XAA, 0X33};
                        if(STATE_BKSCT_IDLE == State_bksct)
                        {
                            memset(fskBuff, 0, sizeof(fskBuff));
                            fskLen = user_fskFrame(fskBuff, sizeof(fskBuff), data, sizeof(data));
                            State_bksct = STATE_BKSCT_BUSY;
                        }
					}
					memset(rxBuf1,0, SENSOR_BYTE);
					
				}
//			Radio->Process( );
		}
		
    switch( Radio->Process( ) )
    {
    case RF_RX_TIMEOUT:
        // Send the next PING frame
        Buffer[0] = '1';
        Buffer[1] = '2';
        Buffer[2] = '3';
        Buffer[3] = '4';
        Buffer[4] = '1';
        Buffer[5] = '2';

				Radio->SetTxPacket( Buffer, BufferSize );
//        for( i = 4; i < BufferSize; i++ )
//        {
//            Buffer[i] = i - 4;
//        }
//        Radio->SetTxPacket( Buffer, BufferSize );
        break;
    case RF_RX_DONE:
        Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
    
        if( BufferSize > 0 )
        {
						HAL_UART_Transmit(&huart1, (uint8_t *)Buffer, strlen(Buffer), 0xffff);
            if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
            {
                // Indicates on a LED that the received frame is a PONG
                //LedToggle( LED_GREEN );

                // Send the next PING frame            
                Buffer[0] = '1';
                Buffer[1] = '2';
                Buffer[2] = '3';
                Buffer[3] = '4';
                // We fill the buffer with numbers for the payload 
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                Radio->SetTxPacket( Buffer, BufferSize );
            }
            else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
            { // A master already exists then become a slave
                EnableMaster = false;
                //LedOff( LED_RED );
            }
        }            
        break;
    case RF_TX_DONE:
        // Indicates on a LED that we have sent a PING
        //LedToggle( LED_RED );
        Radio->StartRx( );
        break;
    default:
        break;
    }
}
#endif


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12)
	{
		static uint8_t i = 0;
		if(i++ % 2 == 0)
		{
				HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
				
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		}
		
		
		
	}
	
}



/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

	#if 0
    if(htim->Instance == TIM4)
    {
        if(State_bksct == STATE_BKSCT_BUSY)
        {
            static u32 cntBit = 0;
            u16 cntByte = cntBit / 8;
            u8 BitAll = 8 * (fskLen + 2);

            u8 *pdata = fskBuff;
        	u8 tempvalue = 0; 

        	if(cntBit < BitAll)
        	{
                u8 iovalue = 0;
        		tempvalue = pdata[cntByte] >> (7 - cntBit % 8);
                iovalue = tempvalue & 0x01;  
                // INVERSE
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, iovalue? 0 : 1);
                cntBit++;
        	}
            else
            {
                // send complete reset state
                cntBit = 0;
                State_bksct = STATE_BKSCT_IDLE;
            }
        }

    }
		#endif
}


/*
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    int i = 0;
		i++;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
		MX_RTC_Init();
}
*/

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
  MX_TIM2_Init();
  //MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	
	//HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	// PWM

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	while(1);
	HAL_Delay(500);
	
	// STANDBY
	
	  /* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
    mainly  when using more than one wakeup source this is to not miss any wakeup event.
    - Disable all used wakeup sources,
    - Clear all related wakeup flags, 
    - Re-enable all used wakeup sources,
    - Enter the Standby mode.
  */
  /* Disable all used wakeup sources*/
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  
  /* Re-enable all used wakeup sources*/
  /* ## Setting the Wake up time ############################################*/
  /*  RTC Wakeup Interrupt Generation:
    Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
    Wakeup Time = Wakeup Time Base * WakeUpCounter 
      = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
      ==> WakeUpCounter = Wakeup Time / Wakeup Time Base
  
    To configure the wake up timer to 4s the WakeUpCounter is set to 0x1FFF:
    RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
    Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
    Wakeup Time = ~4s = 0,410ms  * WakeUpCounter
      ==> WakeUpCounter = ~4s/0,410ms = 9750 = 0x2616 */
			//0.435ms  11495 5s
  //HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x2616, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10345, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	
  /* Clear all related wakeup flags */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  
  /* Enter the Standby mode */
  HAL_PWR_EnterSTANDBYMode();
	
	
	
  while (1)
  {
			
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//OnMaster( );
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  
	/* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
	
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

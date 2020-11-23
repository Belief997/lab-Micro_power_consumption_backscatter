/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <stdint.h>
#include <stdbool.h> 

#include "platform.h"
//#define HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
//                                          uint32_t Timeout)
#if defined( USE_SX1276_RADIO )

#include "ioe.h"
//#include "spi.h"
//#include "stm32l051xx.h"
#include "stm32l152xba.h"
#include "sx1276-Hal.h"
//#include "stm32l0xx_hal.h"
#include "stm32l1xx_hal.h"
extern volatile uint32_t TickCounter;

/*!
 * SX1276 RESET I/O definitions
 */
#if defined( STM32L051xx ) 
#define RESET_IOPORT                                GPIOA
#define RESET_PIN                                   GPIO_PIN_8
#define NSS_IOPORT                                  GPIOB
#define NSS_PIN                                     GPIO_PIN_0
#define DIO0_IOPORT                                 GPIOA
#define DIO0_PIN                                    GPIO_PIN_9
#define DIO1_IOPORT                                 GPIOA
#define DIO1_PIN                                    GPIO_PIN_10
#define DIO2_IOPORT                                 GPIOA
#define DIO2_PIN                                    GPIO_PIN_11
#define DIO3_IOPORT                                 GPIOA
#define DIO3_PIN                                    GPIO_PIN_12
#define DIO4_IOPORT                                 GPIOA
#define DIO4_PIN                                    GPIO_PIN_3
#define DIO5_IOPORT                                 GPIOA
#define DIO5_PIN                                    GPIO_PIN_4
#define LED_IOPORT 																	GPIOA
#define LED_PIN                							        GPIO_PIN_15
#define SENSOR_IOPORT 																	GPIOB
#define SENSOR_PIN																			GPIO_PIN_4
#define GATE_IOPORT																GPIOA
#define GATE_PIN																	GPIO_PIN_1
					
#endif

#if defined( STM32L152xBA ) 
#define RESET_IOPORT                                GPIOB
#define RESET_PIN                                   GPIO_PIN_8
#define NSS_IOPORT                                  GPIOA
#define NSS_PIN                                     GPIO_PIN_4

// set all input
#define DIO0_IOPORT                                 GPIOB
#define DIO0_PIN                                    GPIO_PIN_9
#define DIO1_IOPORT                                 GPIOB
#define DIO1_PIN                                    GPIO_PIN_3
#define DIO2_IOPORT                                 GPIOB
#define DIO2_PIN                                    GPIO_PIN_4
#define DIO3_IOPORT                                 GPIOB
#define DIO3_PIN                                    GPIO_PIN_5
#define DIO4_IOPORT                                 GPIOA
#define DIO4_PIN                                    GPIO_PIN_3
#define DIO5_IOPORT                                 GPIOA
#define DIO5_PIN                                    GPIO_PIN_2

#define LED_IOPORT 									GPIOA
#define LED_PIN                						GPIO_PIN_11
#define SENSOR_IOPORT 								GPIOA
#define SENSOR_PIN									GPIO_PIN_11
#define GATE_IOPORT									GPIOA
#define GATE_PIN									GPIO_PIN_1
					
#endif

#define Bit_RESET GPIO_PIN_RESET
#define Bit_SET GPIO_PIN_SET
#define GPIO_ReadInputDataBit HAL_GPIO_ReadPin
#define GPIO_WriteBit HAL_GPIO_WritePin

extern SPI_HandleTypeDef hspi1;

void tri_sensor(void){
	uint32_t tickstart;
	GPIO_WriteBit(SENSOR_IOPORT, SENSOR_PIN,Bit_RESET);
	tickstart = HAL_GetTick();
	while(HAL_GetTick()-tickstart<=2);
	GPIO_WriteBit(SENSOR_IOPORT, SENSOR_PIN,Bit_SET);
}
void toggle_sensor(void){
	HAL_GPIO_TogglePin( SENSOR_IOPORT, SENSOR_PIN);
}

//void reset_led(void){
//	GPIO_WriteBit( LED_IOPORT, LED_PIN, Bit_RESET );
//}

void toggle_led(void){
	HAL_GPIO_TogglePin(LED_IOPORT, LED_PIN);
}
void set_led(void){
	GPIO_WriteBit( LED_IOPORT, LED_PIN, Bit_SET );
}

void reset_led(void){
	GPIO_WriteBit( LED_IOPORT, LED_PIN, Bit_RESET );
}
uint8_t SpiInOut( uint8_t outData )
{
    /* Send SPIy data */
		uint8_t rx_data;
		//GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
		HAL_SPI_TransmitReceive( &hspi1, &outData, &rx_data,1,50);
		//GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
//    SPI_I2S_SendData( SPI_INTERFACE, outData );
//    while( SPI_I2S_GetFlagStatus( SPI_INTERFACE, SPI_I2S_FLAG_RXNE ) == RESET );
    return rx_data;
}

void SX1276InitIo( void )
{
		GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
		GPIO_WriteBit(SENSOR_IOPORT, SENSOR_PIN,Bit_SET);
		//GPIO_WriteBit(GATE_IOPORT, GATE_PIN,Bit_SET);
		GPIO_WriteBit(GATE_IOPORT, GATE_PIN,Bit_RESET);

	
//    GPIO_InitTypeDef GPIO_InitStructure;

//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
//                            RCC_AHB1Periph_GPIOG, ENABLE );
//#else
//    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
//                            RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE );
//#endif

//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//#else
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//#endif
//    
//    // Configure NSS as output
//    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
//    GPIO_InitStructure.GPIO_Pin = NSS_PIN;
//    GPIO_Init( NSS_IOPORT, &GPIO_InitStructure );

//    // Configure radio DIO as inputs
//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//#else
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//#endif

//    // Configure DIO0
//    GPIO_InitStructure.GPIO_Pin =  DIO0_PIN;
//    GPIO_Init( DIO0_IOPORT, &GPIO_InitStructure );
//    
//    // Configure DIO1
//    GPIO_InitStructure.GPIO_Pin =  DIO1_PIN;
//    GPIO_Init( DIO1_IOPORT, &GPIO_InitStructure );
//    
//    // Configure DIO2
//    GPIO_InitStructure.GPIO_Pin =  DIO2_PIN;
//    GPIO_Init( DIO2_IOPORT, &GPIO_InitStructure );
    
    // REAMARK: DIO3/4/5 configured are connected to IO expander

    // Configure DIO3 as input
    
    // Configure DIO4 as input
    
    // Configure DIO5 as input
}

void SX1276SetReset( uint8_t state )
{
	// reset output state
//    GPIO_InitTypeDef GPIO_InitStructure;

    if( state == RADIO_RESET_ON )
    {
        // Set RESET pin to 0
        GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_RESET );

//        // Configure RESET as output
//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//#else

//        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//#endif        
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_InitStructure.GPIO_Pin = RESET_PIN;
//        GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
    }
    else
    {
//#if FPGA == 0    
//        // Configure RESET as input
//#if defined( STM32F4XX ) || defined( STM32F2XX ) || defined( STM32F429_439xx )
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//#else
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//#endif        
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//        GPIO_InitStructure.GPIO_Pin =  RESET_PIN;
//        GPIO_Init( RESET_IOPORT, &GPIO_InitStructure );
//#else
        GPIO_WriteBit( RESET_IOPORT, RESET_PIN, Bit_SET );
//#endif
    }
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
		
    //NSS = 0;
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );

    SpiInOut( addr | 0x80 );
	
		//HAL_SPI_TransmitReceive( &hspi1, &buffer, &rx_data, size,1);
	
    for( i = 0; i < size; i++ )
    {
        SpiInOut( buffer[i] );
    }

    //NSS = 1;
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_RESET );

    SpiInOut( addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( 0 );
    }

    //NSS = 1;
    GPIO_WriteBit( NSS_IOPORT, NSS_PIN, Bit_SET );
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

inline uint8_t SX1276ReadDio0( void )
{
    return GPIO_ReadInputDataBit( DIO0_IOPORT, DIO0_PIN );
}

inline uint8_t SX1276ReadDio1( void )
{
    return GPIO_ReadInputDataBit( DIO1_IOPORT, DIO1_PIN );
}

inline uint8_t SX1276ReadDio2( void )
{
    return GPIO_ReadInputDataBit( DIO2_IOPORT, DIO2_PIN );
}

inline uint8_t SX1276ReadDio3( void )
{
    return GPIO_ReadInputDataBit( DIO3_IOPORT, DIO3_PIN );
}

inline uint8_t SX1276ReadDio4( void )
{
    return GPIO_ReadInputDataBit( DIO4_IOPORT, DIO4_PIN );
}

inline uint8_t SX1276ReadDio5( void )
{
    return GPIO_ReadInputDataBit( DIO5_IOPORT, DIO5_PIN );
}

inline void SX1276WriteRxTx( uint8_t txEnable )
{
//    if( txEnable != 0 )
//    {
//        GPIO_WriteBit( FEM_CTX_IOPORT, FEM_CTX_PIN, Bit_SET);
//        GPIO_WriteBit( FEM_CPS_IOPORT, FEM_CPS_PIN, Bit_RESET);
//    }
//    else
//    {
//        GPIO_WriteBit( FEM_CTX_IOPORT, FEM_CTX_PIN, Bit_RESET);
//        GPIO_WriteBit( FEM_CPS_IOPORT, FEM_CPS_PIN, Bit_SET);
//    }
}

#endif // USE_SX1276_RADIO

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
 * \file       sx1276-Hal.h
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__

#include "ioe.h"
#include "stm32f10x_type.h"

/*!
 * DIO state read functions mapping
 */
#define DIO0                                        SX1276ReadDio0( )
#define DIO1                                        SX1276ReadDio1( )
#define DIO2                                        SX1276ReadDio2( )
#define DIO3                                        SX1276ReadDio3( )
#define DIO4                                        SX1276ReadDio4( )
#define DIO5                                        SX1276ReadDio5( )

//#define GPIO_PIN_0                 ((uint16_t)0x0001U)  /* Pin 0 selected    */
//#define GPIO_PIN_1                 ((uint16_t)0x0002U)  /* Pin 1 selected    */
//#define GPIO_PIN_2                 ((uint16_t)0x0004U)  /* Pin 2 selected    */
//#define GPIO_PIN_3                 ((uint16_t)0x0008U)  /* Pin 3 selected    */
//#define GPIO_PIN_4                 ((uint16_t)0x0010U)  /* Pin 4 selected    */
//#define GPIO_PIN_5                 ((uint16_t)0x0020U)  /* Pin 5 selected    */
//#define GPIO_PIN_6                 ((uint16_t)0x0040U)  /* Pin 6 selected    */
//#define GPIO_PIN_7                 ((uint16_t)0x0080U)  /* Pin 7 selected    */
//#define GPIO_PIN_8                 ((uint16_t)0x0100U)  /* Pin 8 selected    */
//#define GPIO_PIN_9                 ((uint16_t)0x0200U)  /* Pin 9 selected    */
//#define GPIO_PIN_10                ((uint16_t)0x0400U)  /* Pin 10 selected   */
//#define GPIO_PIN_11                ((uint16_t)0x0800U)  /* Pin 11 selected   */
//#define GPIO_PIN_12                ((uint16_t)0x1000U)  /* Pin 12 selected   */
//#define GPIO_PIN_13                ((uint16_t)0x2000U)  /* Pin 13 selected   */
//#define GPIO_PIN_14                ((uint16_t)0x4000U)  /* Pin 14 selected   */
//#define GPIO_PIN_15                ((uint16_t)0x8000U)  /* Pin 15 selected   */
//#define GPIO_PIN_All               ((uint16_t)0xFFFFU)  /* All pins selected */

// RXTX pin control see errata note
#define RXTX( txEnable )                            SX1276WriteRxTx( txEnable );

#define GET_TICK_COUNT( )                           ( TickCounter )
#define TICK_RATE_MS( ms )                          ( ms )

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState;

/*!
 * \brief Initializes the radio interface I/Os
 */
void SX1276InitIo( void );

/*!
 * \brief Set the radio reset pin state
 * 
 * \param state New reset pin state
 */
void SX1276SetReset( uint8_t state );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void SX1276Write( uint8_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [OUT]: data Register value
 */
void SX1276Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Writes the buffer contents to the radio FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the radio FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Gets the SX1276 DIO0 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1276ReadDio0( void );

/*!
 * \brief Gets the SX1276 DIO1 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1276ReadDio1( void );

/*!
 * \brief Gets the SX1276 DIO2 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1276ReadDio2( void );

/*!
 * \brief Gets the SX1276 DIO3 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
 inline uint8_t SX1276ReadDio3( void );

/*!
 * \brief Gets the SX1276 DIO4 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
 inline uint8_t SX1276ReadDio4( void );

/*!
 * \brief Gets the SX1276 DIO5 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
 inline uint8_t SX1276ReadDio5( void );

/*!
 * \brief Writes the external RxTx pin value
 *
 * \remark see errata note
 *
 * \param [IN] txEnable [1: Tx, 0: Rx]
 */
 inline void SX1276WriteRxTx( uint8_t txEnable );
 void reset_led(void);
 void reset_led(void);
 void toggle_led(void);
 void tri_sensor(void);
 void toggle_sensor(void);
#endif //__SX1276_HAL_H__

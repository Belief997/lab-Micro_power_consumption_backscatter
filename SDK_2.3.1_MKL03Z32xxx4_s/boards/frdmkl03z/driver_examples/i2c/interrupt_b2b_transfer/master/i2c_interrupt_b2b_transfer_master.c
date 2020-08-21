/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*  Standard C Included Files */
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* I2C source clock */
#define I2C_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C_MASTER_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define EXAMPLE_I2C_MASTER_BASEADDR I2C0

#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_BAUDRATE 100000U
#define I2C_DATA_LENGTH 33U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];
i2c_master_handle_t g_m_handle;
volatile bool g_MasterCompletionFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
#define IIC_DEMO


static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}



void delay(void)
{
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
	__asm("NOP"); /* delay */
}

void delay_n(uint16_t time)
{
    volatile uint32_t i = 0;
    for (i = 0; i < time; ++i)
    {
    	delay();
    }
}

#ifndef IIC_DEMO
void user_gpioInit(void)
{
    gpio_pin_config_t config_output_L = {kGPIO_DigitalOutput, 0};
    gpio_pin_config_t config_output_H = {kGPIO_DigitalOutput, 1};
    gpio_pin_config_t config_input    = {kGPIO_DigitalInput, 0};

    /* Init output ENABLE GPIO. */
    // GPIOA
    GPIO_PinInit(GPIOA, 6U, &config_output_H);	 // SDA
    GPIO_PinInit(GPIOA, 5U, &config_output_H);   // SCL

//    GPIO_PinInit(GPIOA, 6U, &config_output_L);	 // SDA
//    GPIO_PinInit(GPIOA, 5U, &config_output_L);   // SCL

    GPIO_PinInit(GPIOA, 3U, &config_output_L);	 // SCL
    GPIO_PinInit(GPIOA, 4U, &config_output_L);   // SDA
    // GPIOB
//    GPIO_PinInit(GPIOB, 3U, &config_output_L);

}

#define SCL_H {GPIO_PinWrite(GPIOA, 5U, 1);}
#define SCL_L {GPIO_PinWrite(GPIOA, 5U, 0);}
#define SDA_H {GPIO_PinWrite(GPIOA, 6U, 1);}
#define SDA_L {GPIO_PinWrite(GPIOA, 6U, 0);}
#define SDA_I {gpio_pin_config_t config_input    = {kGPIO_DigitalInput, 0}; GPIO_PinInit(GPIOA, 6U, &config_input);}
#define SDA_O_H {gpio_pin_config_t config_output_H = {kGPIO_DigitalOutput, 1}; GPIO_PinInit(GPIOA, 6U, &config_output_H);}
#define SDA_O_L {gpio_pin_config_t config_output_L = {kGPIO_DigitalOutput, 0}; GPIO_PinInit(GPIOA, 6U, &config_output_L);}
#define READ_SDA GPIO_PinRead(GPIOA, 6U)

// sclk T_h/T_l should be 5us

void IIC_Start(void)
{
	SDA_O_H;
	SCL_H;
	delay_n(4);
	SDA_L;		//START:when CLK is high,DATA change form high to low
	delay_n(4);
	SCL_L;		//钳住I2C总线，准备发送或接收数据
}

void IIC_Byte_Start(void)
{
	SDA_O_L;
	SCL_L;
	delay_n(4);
	SDA_H;		//START:when CLK is high,DATA change form high to low
	delay_n(4);
	SDA_L;
	SCL_L;		//钳住I2C总线，准备发送或接收数据
}

void IIC_Stop(void)
{
	SDA_O_L;		//sda线输出
	SCL_L;
	//STOP:when CLK is high DATA change form low to high
	delay_n(4);
	SCL_H;
	SDA_H;//发送I2C总线结束信号
	delay_n(4);
}

void IIC_Ack(void)
{
	SCL_L;
	SDA_O_L;
	delay_n(2);
	SCL_H;
	delay_n(2);
	SCL_L;
}
//不产生ACK应答
void IIC_NAck(void)
{
	SCL_L;
	SDA_O_H;
	delay_n(2);
	SCL_H;
	delay_n(2);
	SCL_L;
}

uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_I;      //SDA设置为输入
	delay_n(1);
	SCL_H;delay_n(2);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;//时钟输出0
	return 0;
}
void IIC_Send_Byte(uint8_t txd)
{
	uint8_t t;
    SDA_O_L;
	SCL_L;	//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
    	if((txd&0x80)>>7)
    	{
    		SDA_H;
    	}
    	else
    	{
    		SDA_L;
    	}
        txd<<=1;
        delay_n(2);
        SCL_H;
		delay_n(2);
		SCL_L;
		delay_n(2);
    }
}

uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_I;				//SDA设置为输入
    for(i=0;i<8;i++ )
	{
    	SCL_L;
        delay_n(2);
        SCL_H;
        receive<<=1;
        if(READ_SDA)receive++;
        delay_n(1);
    }
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}

void IIC_SEND()
{
	unsigned char cnt_byte=0, i = 0;
	unsigned char length = 2;

	IIC_Start();
	// device addr
	IIC_Send_Byte(0x7E<<1);
	// wait ack
	IIC_Wait_Ack;

	IIC_Byte_Start;
	// sub addr
	IIC_Send_Byte(0x01);
	// wait ack
	IIC_Wait_Ack;

	IIC_Byte_Start;
	// length
	IIC_Send_Byte(length);
	// wait ack
	IIC_Wait_Ack;

	for(cnt_byte = 0; cnt_byte < length; cnt_byte ++)
	{
		IIC_Byte_Start;
		IIC_Send_Byte(cnt_byte);
		// wait ack
		IIC_Wait_Ack;
	}
	IIC_Stop();
}

void IIC_REC()
{
	unsigned char cnt_byte=0, i = 0;
//	unsigned char length = 32;
	unsigned char length = 2;

	IIC_Start();
	// device addr
	IIC_Send_Byte(0x7E<<1);
	// wait ack
	IIC_Wait_Ack;

	IIC_Byte_Start;
	// sub addr
	IIC_Send_Byte(0x01);
	// wait ack
	IIC_Wait_Ack;

	IIC_Byte_Start;
	// length
	IIC_Send_Byte(length);
	// wait ack
	IIC_Wait_Ack;

	for(cnt_byte = 0; cnt_byte < length; cnt_byte ++)
	{
		IIC_Byte_Start;
		IIC_Send_Byte(cnt_byte);
		// wait ack
		IIC_Wait_Ack;
	}

}



/*!
 * @brief Main function
 */
int main(void)
{
    i2c_master_config_t masterConfig;
    uint32_t sourceClock;
    i2c_master_transfer_t masterXfer;

    BOARD_InitPins();
    BOARD_I2C_ConfigurePins();
    user_gpioInit();

    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\nI2C board2board interrupt example -- Master transfer.\r\n");

    /* Set up i2c master to send data to slave*/
    /* First data in txBuff is data length of the transmiting data. */
    g_master_txBuff[0] = I2C_DATA_LENGTH - 1U;
    for (uint32_t i = 1U; i < I2C_DATA_LENGTH; i++)
    {
        g_master_txBuff[i] = i - 1;
    }

    PRINTF("Master will send data :");
    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1U; i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", g_master_txBuff[i + 1]);
    }
    PRINTF("\r\n\r\n");


    IIC_SEND();





//    /*
//     * masterConfig->baudRate_Bps = 100000U;
//     * masterConfig->enableStopHold = false;
//     * masterConfig->glitchFilterWidth = 0U;
//     * masterConfig->enableMaster = true;
//     */
//    I2C_MasterGetDefaultConfig(&masterConfig);
//    masterConfig.baudRate_Bps = I2C_BAUDRATE;
//
//    sourceClock = I2C_MASTER_CLK_FREQ;
//
//    I2C_MasterInit(EXAMPLE_I2C_MASTER_BASEADDR, &masterConfig, sourceClock);
//
//    memset(&g_m_handle, 0, sizeof(g_m_handle));
//    memset(&masterXfer, 0, sizeof(masterXfer));
//
//    /* subAddress = 0x01, data = g_master_txBuff - write to slave.
//      start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
//    uint8_t deviceAddress = 0x01U;
//    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;//  <<1
//    masterXfer.direction = kI2C_Write;
//    masterXfer.subaddress = (uint32_t)deviceAddress;
//    masterXfer.subaddressSize = 1;
//    masterXfer.data = g_master_txBuff;
//    masterXfer.dataSize = I2C_DATA_LENGTH;
//    masterXfer.flags = kI2C_TransferDefaultFlag;
//
//    I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
//    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, &masterXfer);
//
//    /*  Wait for transfer completed. */
//    while (!g_MasterCompletionFlag)
//    {
//    }
//    g_MasterCompletionFlag = false;

    PRINTF("Receive sent data from slave :");

    while(1);


//    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
//      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
//    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
//    masterXfer.direction = kI2C_Read;
//    masterXfer.subaddress = (uint32_t)deviceAddress;
//    masterXfer.subaddressSize = 1;
//    masterXfer.data = g_master_rxBuff;
//    masterXfer.dataSize = I2C_DATA_LENGTH - 1;
//    masterXfer.flags = kI2C_TransferDefaultFlag;
//
//    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, &masterXfer);
//
//    /*  Reset master completion flag to false. */
//    g_MasterCompletionFlag = false;
//
//    /*  Wait for transfer completed. */
//    while (!g_MasterCompletionFlag)
//    {
//    }
//    g_MasterCompletionFlag = false;
//
//    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1; i++)
//    {
//        if (i % 8 == 0)
//        {
//            PRINTF("\r\n");
//        }
//        PRINTF("0x%2x  ", g_master_rxBuff[i]);
//    }
//    PRINTF("\r\n\r\n");
//
//    /* Transfer completed. Check the data.*/
//    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1; i++)
//    {
//        if (g_master_rxBuff[i] != g_master_txBuff[i + 1])
//        {
//            PRINTF("\r\nError occured in the transfer ! \r\n");
//            break;
//        }
//    }
//
//    PRINTF("\r\nEnd of I2C example .\r\n");
//    while (1)
//    {
//    }
}
#endif

#ifdef IIC_DEMO

/*!
 * @brief Main function
 */
int main(void)
{
    i2c_master_config_t masterConfig;
    uint32_t sourceClock;
    i2c_master_transfer_t masterXfer;

    BOARD_InitPins();
    BOARD_I2C_ConfigurePins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\nI2C board2board interrupt example -- Master transfer.\r\n");

    /* Set up i2c master to send data to slave*/
    /* First data in txBuff is data length of the transmiting data. */
    g_master_txBuff[0] = I2C_DATA_LENGTH - 1U;
    for (uint32_t i = 1U; i < I2C_DATA_LENGTH; i++)
    {
        g_master_txBuff[i] = i - 1;
    }

    PRINTF("Master will send data :");
    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1U; i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", g_master_txBuff[i + 1]);
    }
    PRINTF("\r\n\r\n");

    /*
     * masterConfig->baudRate_Bps = 100000U;
     * masterConfig->enableStopHold = false;
     * masterConfig->glitchFilterWidth = 0U;
     * masterConfig->enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = I2C_MASTER_CLK_FREQ;

    I2C_MasterInit(EXAMPLE_I2C_MASTER_BASEADDR, &masterConfig, sourceClock);

    memset(&g_m_handle, 0, sizeof(g_m_handle));
    memset(&masterXfer, 0, sizeof(masterXfer));

    /* subAddress = 0x01, data = g_master_txBuff - write to slave.
      start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop*/
    uint8_t deviceAddress = 0x01U;
    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;//  <<1
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = g_master_txBuff;
    masterXfer.dataSize = I2C_DATA_LENGTH;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, &masterXfer);

    /*  Wait for transfer completed. */
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;

    while(1)
    {
    	I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, &masterXfer);
        while (!g_MasterCompletionFlag)
        {
        }
        g_MasterCompletionFlag = false;

        delay_n(99999999);
        PRINTF("MASTER sent data TO slave Once...\r\n");
    }





//    PRINTF("Receive sent data from slave :");

//    while(1);


//    /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
//      start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
//    masterXfer.slaveAddress = I2C_MASTER_SLAVE_ADDR_7BIT;
//    masterXfer.direction = kI2C_Read;
//    masterXfer.subaddress = (uint32_t)deviceAddress;
//    masterXfer.subaddressSize = 1;
//    masterXfer.data = g_master_rxBuff;
//    masterXfer.dataSize = I2C_DATA_LENGTH - 1;
//    masterXfer.flags = kI2C_TransferDefaultFlag;
//
//    I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER_BASEADDR, &g_m_handle, &masterXfer);
//
//    /*  Reset master completion flag to false. */
//    g_MasterCompletionFlag = false;
//
//    /*  Wait for transfer completed. */
//    while (!g_MasterCompletionFlag)
//    {
//    }
//    g_MasterCompletionFlag = false;
//
//    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1; i++)
//    {
//        if (i % 8 == 0)
//        {
//            PRINTF("\r\n");
//        }
//        PRINTF("0x%2x  ", g_master_rxBuff[i]);
//    }
//    PRINTF("\r\n\r\n");
//
//    /* Transfer completed. Check the data.*/
//    for (uint32_t i = 0U; i < I2C_DATA_LENGTH - 1; i++)
//    {
//        if (g_master_rxBuff[i] != g_master_txBuff[i + 1])
//        {
//            PRINTF("\r\nError occured in the transfer ! \r\n");
//            break;
//        }
//    }
//
//    PRINTF("\r\nEnd of I2C example .\r\n");
//    while (1)
//    {
//    }
}
#endif

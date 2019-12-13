/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
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
 *
 */

#include "AT24C02.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : AT24C02_I2CInit
 * Description   : Initialize the I2C transfer.
 * EEPROM AT24C02 is controlled by I2C, using I2C transfer can access the AT24C02 register. 
 *END**************************************************************************/
at24c02_status_t AT24C02_I2CInit(at24c02_handler_t *handler)
{
    /* The master structure initialize */
    I2C_DRV_MasterInit(handler->i2c_instance, &handler->state);
    /* Configure the device info of I2C */
    handler->device.baudRate_kbps = 100;
    handler->device.address = AT24C02_I2C_ADDR;
    I2C_DRV_MasterSetBaudRate(handler->i2c_instance, &handler->device);
    return kStatus_AT24C02_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : AT24C02_ByteWrite
 * Description   : Byte write to AT24C02 memory.
 * The writing process is through I2C.
 *END**************************************************************************/
at24c02_status_t AT24C02_ByteWrite(at24c02_handler_t *handler, uint8_t address, uint8_t data)
{
    i2c_device_t *device = &(handler->device);   	
    uint8_t retval = 0;
	
    retval = I2C_DRV_MasterSendDataBlocking(handler->i2c_instance,device,&address,1,&data,1,OSA_WAIT_FOREVER);
    if(retval != kStatus_I2C_Success)
    {
        return kStatus_AT24C02_I2CFail;
    }
    return kStatus_AT24C02_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : AT24C02_ACKPoll
 * Description   : ACK polling for byte write.
 * The writing process is through I2C.
 *END**************************************************************************/
at24c02_status_t AT24C02_ACKPoll(at24c02_handler_t *handler)
{
    i2c_device_t *device = &(handler->device);
    uint8_t retval = 0;
    uint8_t byteToSend = 0xFF;
    
    do
    {
        retval = I2C_DRV_MasterSendDataBlocking(handler->i2c_instance,device,NULL,0,&byteToSend,1,OSA_WAIT_FOREVER);
    } while(retval == kStatus_I2C_ReceivedNak);
	
    return kStatus_AT24C02_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : AT24C02_ByteRead
 * Description   : Byte read from AT24C02 memory.
 * The reading process is through I2C.
 *END**************************************************************************/
at24c02_status_t AT24C02_ByteRead(at24c02_handler_t *handler, uint8_t address, uint8_t *data)
{
    i2c_device_t *device = &(handler->device);   
    uint8_t retval = 0;
	
    retval = I2C_DRV_MasterReceiveDataBlocking(handler->i2c_instance,device,&address,1,data,1,OSA_WAIT_FOREVER);
    if(retval != kStatus_I2C_Success)
    {
        return kStatus_AT24C02_I2CFail;
    }
    return kStatus_AT24C02_Success;
} 

/******************************************************************************/

/******************************************************************************
 * EOF
 ******************************************************************************/

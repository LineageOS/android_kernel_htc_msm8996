/*******************************************************************************
Copyright © 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/**
 * @file VL53L0_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */
#include "inc/vl53l0_platform.h"
#include "inc/vl53l0_i2c.h"
#include "inc/vl53l0_api.h"


#define I2C_BUFFER_CONFIG 1
/** Maximum buffer size to be used in i2c */
#define VL53L0_MAX_I2C_XFER_SIZE   64 /* Maximum buffer size to be used in i2c */

#if I2C_BUFFER_CONFIG == 0
    
    uint8_t i2c_global_buffer[VL53L0_MAX_I2C_XFER_SIZE];

    #define DECL_I2C_BUFFER
    #define VL53L0_GetLocalBuffer(Dev, n_byte)  i2c_global_buffer

#elif I2C_BUFFER_CONFIG == 1
    
    #define DECL_I2C_BUFFER  uint8_t LocBuffer[VL53L0_MAX_I2C_XFER_SIZE];
    #define VL53L0_GetLocalBuffer(Dev, n_byte)  LocBuffer
#elif I2C_BUFFER_CONFIG == 2
    
    #define DECL_I2C_BUFFER
#else
#error "invalid I2C_BUFFER_CONFIG "
#endif


#define VL53L0_I2C_USER_VAR         
#define VL53L0_GetI2CAccess(Dev)    
#define VL53L0_DoneI2CAcces(Dev)    


VL53L0_Error VL53L0_LockSequenceAccess(VL53L0_DEV Dev){
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    return Status;
}

VL53L0_Error VL53L0_UnlockSequenceAccess(VL53L0_DEV Dev){
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    return Status;
}


VL53L0_Error VL53L0_ReadMulti(VL53L0_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){
    VL53L0_I2C_USER_VAR
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    if (count>=VL53L0_MAX_I2C_XFER_SIZE){
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }
	
    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0_read_multi(deviceAddress, index, pdata, count);

	if (status_int != 0)
		Status = VL53L0_ERROR_CONTROL_INTERFACE;
	
    return Status;
}


VL53L0_Error VL53L0_WrByte(VL53L0_DEV Dev, uint8_t index, uint8_t data){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0_write_byte(deviceAddress, index, data);

	if (status_int != 0)
		Status = VL53L0_ERROR_CONTROL_INTERFACE;
	
    return Status;
}

VL53L0_Error VL53L0_WrWord(VL53L0_DEV Dev, uint8_t index, uint16_t data){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0_write_word(deviceAddress, index, data);

	if (status_int != 0)
		Status = VL53L0_ERROR_CONTROL_INTERFACE;
	
    return Status;
}

VL53L0_Error VL53L0_WrDWord(VL53L0_DEV Dev, uint8_t index, uint32_t data){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0_write_dword(deviceAddress, index, data);

	if (status_int != 0)
		Status = VL53L0_ERROR_CONTROL_INTERFACE;
	
    return Status;
}

VL53L0_Error VL53L0_UpdateByte(VL53L0_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;
    uint8_t data;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0_read_byte(deviceAddress, index, &data);

    if (status_int != 0)
        Status = VL53L0_ERROR_CONTROL_INTERFACE;

    if (Status == VL53L0_ERROR_NONE) {
        data = (data & AndData) | OrData;
        status_int = VL53L0_write_byte(deviceAddress, index, data);

        if (status_int != 0)
            Status = VL53L0_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53L0_Error VL53L0_RdByte(VL53L0_DEV Dev, uint8_t index, uint8_t *data){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0_read_byte(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0_Error VL53L0_RdWord(VL53L0_DEV Dev, uint8_t index, uint16_t *data){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0_read_word(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0_Error  VL53L0_RdDWord(VL53L0_DEV Dev, uint8_t index, uint32_t *data){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0_read_dword(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0_ERROR_CONTROL_INTERFACE;

    return Status;
}

#define VL53L0_POLLINGDELAY_LOOPNB  250
VL53L0_Error VL53L0_PollingDelay(VL53L0_DEV Dev){
    VL53L0_Error status = VL53L0_ERROR_NONE;
    volatile uint32_t i;

    for(i=0;i<VL53L0_POLLINGDELAY_LOOPNB;i++){
        
        asm("nop");
    }

    return status;
}

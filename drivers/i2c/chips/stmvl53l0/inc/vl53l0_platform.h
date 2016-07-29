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


#ifndef _VL53L0_PLATFORM_H_
#define _VL53L0_PLATFORM_H_

#include "vl53l0_def.h"
#include "vl53l0_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

 

typedef struct {
    VL53L0_DevData_t Data;               

    
    uint8_t   I2cDevAddr;                
    uint8_t   comms_type;                
    uint16_t  comms_speed_khz;           

} VL53L0_Dev_t;


typedef VL53L0_Dev_t* VL53L0_DEV;

#define PALDevDataGet(Dev, field) (Dev->Data.field)

#define PALDevDataSet(Dev, field, data) (Dev->Data.field)=(data)



VL53L0_Error VL53L0_LockSequenceAccess(VL53L0_DEV Dev);

VL53L0_Error VL53L0_UnlockSequenceAccess(VL53L0_DEV Dev);


/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0_Error
 */

VL53L0_Error VL53L0_ReadMulti(VL53L0_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);

VL53L0_Error VL53L0_WrByte(VL53L0_DEV Dev, uint8_t index, uint8_t data);

VL53L0_Error VL53L0_WrWord(VL53L0_DEV Dev, uint8_t index, uint16_t data);

VL53L0_Error VL53L0_WrDWord(VL53L0_DEV Dev, uint8_t index, uint32_t data);

VL53L0_Error VL53L0_RdByte(VL53L0_DEV Dev, uint8_t index, uint8_t *data);

VL53L0_Error VL53L0_RdWord(VL53L0_DEV Dev, uint8_t index, uint16_t *data);

VL53L0_Error VL53L0_RdDWord(VL53L0_DEV Dev, uint8_t index, uint32_t *data);

VL53L0_Error VL53L0_UpdateByte(VL53L0_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData);


    
VL53L0_Error VL53L0_PollingDelay(VL53L0_DEV Dev); 


#endif  




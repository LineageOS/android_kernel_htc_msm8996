/*******************************************************************************
 Copyright © 2016, STMicroelectronics International N.V.
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
 ******************************************************************************/

#include "inc/vl53l0_api.h"
#include "inc/vl53l0_api_core.h"
#include "inc/vl53l0_api_strings.h"

#ifndef __KERNEL__
#include <stdlib.h>
#endif

#if 0
#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(TRACE_MODULE_API, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(TRACE_MODULE_API, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(TRACE_MODULE_API, status, fmt, ##__VA_ARGS__)
#endif

VL53L0_Error VL53L0_check_part_used(VL53L0_DEV Dev,
		uint8_t *Revision,
		VL53L0_DeviceInfo_t *pVL53L0_DeviceInfo)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	uint8_t ModuleIdInt;
	char *ProductId_tmp;

	

	Status = VL53L0_get_info_from_device(Dev, 2);

	if (Status == VL53L0_ERROR_NONE) {
		ModuleIdInt = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, ModuleId);

	if (ModuleIdInt == 0) {
		*Revision = 0;
		
	} else {
		*Revision = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, Revision);
		ProductId_tmp = VL53L0_GETDEVICESPECIFICPARAMETER(Dev,
			ProductId);
		
	}
	}

	
	return Status;
}


VL53L0_Error VL53L0_get_device_info(VL53L0_DEV Dev,
				VL53L0_DeviceInfo_t *pVL53L0_DeviceInfo)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	uint8_t revision_id;
	uint8_t Revision;

	Status = VL53L0_check_part_used(Dev, &Revision, pVL53L0_DeviceInfo);

	if (Status == VL53L0_ERROR_NONE) {
		if (Revision == 0) {
			
					
		} else if ((Revision <= 34) && (Revision != 32)) {
			
					
		} else if (Revision < 39) {
			
					
		} else {
			
					
		}

		
				

	}

	if (Status == VL53L0_ERROR_NONE) {
		Status = VL53L0_RdByte(Dev, VL53L0_REG_IDENTIFICATION_MODEL_ID,
				&pVL53L0_DeviceInfo->ProductType);
	}

	if (Status == VL53L0_ERROR_NONE) {
		Status = VL53L0_RdByte(Dev,
			VL53L0_REG_IDENTIFICATION_REVISION_ID,
				&revision_id);
		pVL53L0_DeviceInfo->ProductRevisionMajor = 1;
		pVL53L0_DeviceInfo->ProductRevisionMinor =
					(revision_id & 0xF0) >> 4;
	}

	return Status;
}


VL53L0_Error VL53L0_get_device_error_string(VL53L0_DeviceError ErrorCode,
		char *pDeviceErrorString)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	

	switch (ErrorCode) {
	case VL53L0_DEVICEERROR_NONE:
		
			
	break;
	case VL53L0_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
		
			
	break;
	case VL53L0_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
		
			
	break;
	case VL53L0_DEVICEERROR_NOVHVVALUEFOUND:
		
			
	break;
	case VL53L0_DEVICEERROR_MSRCNOTARGET:
		
			
	break;
	case VL53L0_DEVICEERROR_SNRCHECK:
		
			
	break;
	case VL53L0_DEVICEERROR_RANGEPHASECHECK:
		
			
	break;
	case VL53L0_DEVICEERROR_SIGMATHRESHOLDCHECK:
		
			
	break;
	case VL53L0_DEVICEERROR_TCC:
		
			
	break;
	case VL53L0_DEVICEERROR_PHASECONSISTENCY:
		
			
	break;
	case VL53L0_DEVICEERROR_MINCLIP:
		
			
	break;
	case VL53L0_DEVICEERROR_RANGECOMPLETE:
		
			
	break;
	case VL53L0_DEVICEERROR_ALGOUNDERFLOW:
		
			
	break;
	case VL53L0_DEVICEERROR_ALGOOVERFLOW:
		
			
	break;
	case VL53L0_DEVICEERROR_RANGEIGNORETHRESHOLD:
		
			
	break;

	
		
			

	}

	
	return Status;
}

VL53L0_Error VL53L0_get_range_status_string(uint8_t RangeStatus,
		char *pRangeStatusString)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	

	switch (RangeStatus) {
	case 0:
		
			
	break;
	case 1:
		
			
	break;
	case 2:
		
			
	break;
	case 3:
		
			
	break;
	case 4:
		
			
	break;
	case 5:
		
			
	break;

	
		
				
	}

	
	return Status;
}

VL53L0_Error VL53L0_get_pal_error_string(VL53L0_Error PalErrorCode,
		char *pPalErrorString)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	

	switch (PalErrorCode) {
	case VL53L0_ERROR_NONE:
		
			
	break;
	case VL53L0_ERROR_CALIBRATION_WARNING:
		
			
	break;
	case VL53L0_ERROR_MIN_CLIPPED:
		
			
	break;
	case VL53L0_ERROR_UNDEFINED:
		
			
	break;
	case VL53L0_ERROR_INVALID_PARAMS:
		
			
	break;
	case VL53L0_ERROR_NOT_SUPPORTED:
		
			
	break;
	case VL53L0_ERROR_RANGE_ERROR:
		
			
	break;
	case VL53L0_ERROR_TIME_OUT:
		
			
	break;
	case VL53L0_ERROR_MODE_NOT_SUPPORTED:
		
			
	break;
	case VL53L0_ERROR_BUFFER_TOO_SMALL:
		
			
	break;
	case VL53L0_ERROR_GPIO_NOT_EXISTING:
		
			
	break;
	case VL53L0_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED:
		
			
	break;
	case VL53L0_ERROR_CONTROL_INTERFACE:
		
			
	break;
	case VL53L0_ERROR_INVALID_COMMAND:
		
			
	break;
	case VL53L0_ERROR_DIVISION_BY_ZERO:
		
			
	break;
	case VL53L0_ERROR_REF_SPAD_INIT:
		
			
	break;
	case VL53L0_ERROR_NOT_IMPLEMENTED:
		
			
	break;

	
		
				
	}

	
	return Status;
}

VL53L0_Error VL53L0_get_pal_state_string(VL53L0_State PalStateCode,
		char *pPalStateString)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	

	switch (PalStateCode) {
	case VL53L0_STATE_POWERDOWN:
		
			
	break;
	case VL53L0_STATE_WAIT_STATICINIT:
		
			
	break;
	case VL53L0_STATE_STANDBY:
		
			
	break;
	case VL53L0_STATE_IDLE:
		
			
	break;
	case VL53L0_STATE_RUNNING:
		
			
	break;
	case VL53L0_STATE_UNKNOWN:
		
			
	break;
	case VL53L0_STATE_ERROR:
		
			
	break;

	
		
			
	}

	
	return Status;
}

VL53L0_Error VL53L0_get_sequence_steps_info(
		VL53L0_SequenceStepId SequenceStepId,
		char *pSequenceStepsString)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	

	switch (SequenceStepId) {
	case VL53L0_SEQUENCESTEP_TCC:
		
			
	break;
	case VL53L0_SEQUENCESTEP_DSS:
		
			
	break;
	case VL53L0_SEQUENCESTEP_MSRC:
		
			
	break;
	case VL53L0_SEQUENCESTEP_PRE_RANGE:
		
			
	break;
	case VL53L0_SEQUENCESTEP_FINAL_RANGE:
		
			
	break;

	
		Status = VL53L0_ERROR_INVALID_PARAMS;
	}

	

	return Status;
}


VL53L0_Error VL53L0_get_limit_check_info(VL53L0_DEV Dev, uint16_t LimitCheckId,
	char *pLimitCheckString)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	

	switch (LimitCheckId) {
	case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
		
			
	break;
	case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
		
			
	break;
	case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:
		
			
	break;
	case VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
		
			
	break;

	
		
			

	}

	
	return Status;
}

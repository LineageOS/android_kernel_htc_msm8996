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

#include "inc/vl53l0_api.h"
#include "inc/vl53l0_tuning.h"
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/string.h>



#define VL53L0_COPYSTRING(str, ...) strcpy(str, "aaa")


#define VL53L0_SETPARAMETERFIELD(Dev, field, value) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = PALDevDataGet(Dev, CurrentParameters); \
        CurrentParameters.field = value; \
        CurrentParameters = \
            PALDevDataSet(Dev, CurrentParameters, \
                          CurrentParameters); }
#define VL53L0_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = PALDevDataGet(Dev, CurrentParameters); \
        CurrentParameters.field[index] = value; \
        CurrentParameters = \
            PALDevDataSet(Dev, CurrentParameters, \
                          CurrentParameters); }

#define VL53L0_GETPARAMETERFIELD(Dev, field, variable) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = \
            PALDevDataGet(Dev, CurrentParameters);  \
        variable = CurrentParameters.field; }
#define VL53L0_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = \
            PALDevDataGet(Dev, CurrentParameters);      \
        variable = CurrentParameters.field[index]; }

#define VL53L0_SETDEVICESPECIFICPARAMETER(Dev, field, value) \
	if(Status==VL53L0_ERROR_NONE){ \
		DeviceSpecificParameters = \
            PALDevDataGet(Dev, DeviceSpecificParameters);   \
		DeviceSpecificParameters.field = value; \
		DeviceSpecificParameters = \
            PALDevDataSet(Dev, DeviceSpecificParameters, \
                          DeviceSpecificParameters); }
#define VL53L0_GETDEVICESPECIFICPARAMETER(Dev, field) \
		PALDevDataGet(Dev, DeviceSpecificParameters).field

#define VL53L0_FIXPOINT1616TOFIXPOINT97(Value) \
            (uint16_t)((Value>>9)&0xFFFF)
#define VL53L0_FIXPOINT97TOFIXPOINT1616(Value) \
            (FixPoint1616_t)(Value<<9)

#define VL53L0_FIXPOINT1616TOFIXPOINT88(Value) \
            (uint16_t)((Value>>8)&0xFFFF)
#define VL53L0_FIXPOINT88TOFIXPOINT1616(Value) \
            (FixPoint1616_t)(Value<<8)

#define VL53L0_FIXPOINT1616TOFIXPOINT412(Value) \
            (uint16_t)((Value>>4)&0xFFFF)
#define VL53L0_FIXPOINT412TOFIXPOINT1616(Value) \
            (FixPoint1616_t)(Value<<4)

#define VL53L0_FIXPOINT1616TOFIXPOINT313(Value) \
            (uint16_t)((Value>>3)&0xFFFF)
#define VL53L0_FIXPOINT313TOFIXPOINT1616(Value) \
            (FixPoint1616_t)(Value<<3)

#define VL53L0_FIXPOINT1616TOFIXPOINT08(Value) \
            (uint8_t)((Value>>8)&0x00FF)
#define VL53L0_FIXPOINT08TOFIXPOINT1616(Value) \
            (FixPoint1616_t)(Value<<8)

#define VL53L0_FIXPOINT1616TOFIXPOINT53(Value) \
            (uint8_t)((Value>>13)&0x00FF)
#define VL53L0_FIXPOINT53TOFIXPOINT1616(Value) \
            (FixPoint1616_t)(Value<<13)

#define VL53L0_FIXPOINT1616TOFIXPOINT102(Value) \
            (uint16_t)((Value>>14)&0x0FFF)
#define VL53L0_FIXPOINT102TOFIXPOINT1616(Value) \
            (FixPoint1616_t)(Value<<12)

#define VL53L0_MAKEUINT16(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + \
		(uint16_t)lsb)

#define REF_ARRAY_SPAD_0  0
#define REF_ARRAY_SPAD_5  5
#define REF_ARRAY_SPAD_10 10

uint32_t refArrayQuadrants[4] = {REF_ARRAY_SPAD_10, REF_ARRAY_SPAD_5,
		REF_ARRAY_SPAD_0, REF_ARRAY_SPAD_5 };

uint8_t VL53L0_encode_vcsel_period(uint8_t vcsel_period_pclks);
uint8_t VL53L0_decode_vcsel_period(uint8_t vcsel_period_reg);
VL53L0_Error VL53L0_check_part_used(VL53L0_DEV Dev, uint8_t* Revision,
		VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo);
VL53L0_Error VL53L0_get_info_from_device(VL53L0_DEV Dev, uint8_t option);
VL53L0_Error VL53L0_device_read_strobe(VL53L0_DEV Dev);
VL53L0_Error VL53L0_get_pal_range_status(VL53L0_DEV Dev,
					 uint8_t DeviceRangeStatus,
					 FixPoint1616_t SignalRate,
					 uint16_t EffectiveSpadRtnCount,
					 VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
					 uint8_t* pPalRangeStatus);
VL53L0_Error VL53L0_measurement_poll_for_completion(VL53L0_DEV Dev);
VL53L0_Error VL53L0_confirm_measurement_start(VL53L0_DEV Dev);
VL53L0_Error VL53L0_start_histogram_measurement(VL53L0_DEV Dev,
		VL53L0_HistogramModes histoMode, uint32_t count);
VL53L0_Error VL53L0_read_histo_measurement(VL53L0_DEV Dev, uint32_t *histoData,
		uint32_t offset, VL53L0_HistogramModes histoMode);
VL53L0_Error VL53L0_reverse_bytes(uint8_t *data, uint32_t size);
VL53L0_Error VL53L0_load_tuning_settings(VL53L0_DEV Dev,
		uint8_t* pTuningSettingBuffer);
VL53L0_Error enable_ref_spads(VL53L0_DEV Dev,
                              uint8_t apertureSpads,
                              uint8_t goodSpadArray[],
                              uint8_t spadArray[],
                              uint32_t size,
                              uint32_t start,
                              uint32_t offset,
                              uint32_t minimumSpadCount,
                              uint32_t *lastSpad);
VL53L0_Error perform_ref_signal_measurement(VL53L0_DEV Dev,
		uint16_t *refSignalRate);
uint8_t is_aperture(uint32_t spadIndex);
void get_next_good_spad(uint8_t array1[], uint32_t size, uint32_t current,
		int32_t *next);
VL53L0_Error enable_spad_bit(uint8_t spadArray[], uint32_t size,
		uint32_t spadIndex);
VL53L0_Error set_ref_spad_map(VL53L0_DEV Dev, uint8_t *refSpadArray);
VL53L0_Error get_ref_spad_map(VL53L0_DEV Dev, uint8_t *refSpadArray);
VL53L0_Error count_enabled_spads(uint8_t spadArray[],
        uint32_t byteCount, uint32_t maxSpads, uint32_t *pTotalSpadsEnabled, uint8_t *pIsAperture);
VL53L0_Error sequence_step_enabled(VL53L0_DEV Dev,
                                   VL53L0_SequenceStepId SequenceStepId,
                                   uint8_t SequenceConfig,
                                   uint8_t* pSequenceStepEnabled);
VL53L0_Error set_sequence_step_timeout(VL53L0_DEV Dev,
                                       VL53L0_SequenceStepId SequenceStepId,
                                       FixPoint1616_t TimeOutMilliSecs);
VL53L0_Error get_sequence_step_timeout(VL53L0_DEV Dev,
                                       VL53L0_SequenceStepId SequenceStepId,
                                       FixPoint1616_t *pTimeOutMilliSecs);

VL53L0_Error VL53L0_GetVersion(VL53L0_Version_t* pVersion) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    pVersion->major = VL53L0_IMPLEMENTATION_VER_MAJOR;
    pVersion->minor = VL53L0_IMPLEMENTATION_VER_MINOR;
    pVersion->build = VL53L0_IMPLEMENTATION_VER_SUB;

    pVersion->revision = VL53L0_IMPLEMENTATION_VER_REVISION;

    return Status;
}

VL53L0_Error VL53L0_GetPalSpecVersion(VL53L0_Version_t* pPalSpecVersion)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    pPalSpecVersion->major = VL53L0_SPECIFICATION_VER_MAJOR;
    pPalSpecVersion->minor = VL53L0_SPECIFICATION_VER_MINOR;
    pPalSpecVersion->build = VL53L0_SPECIFICATION_VER_SUB;

    pPalSpecVersion->revision = VL53L0_SPECIFICATION_VER_REVISION;

    return Status;
}

VL53L0_Error VL53L0_GetProductRevision(VL53L0_DEV Dev,
		uint8_t* pProductRevisionMajor,
		uint8_t* pProductRevisionMinor) {

    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t revision_id;


	Status = VL53L0_RdByte(Dev, VL53L0_REG_IDENTIFICATION_REVISION_ID,
			&revision_id);
	*pProductRevisionMajor = 1;
	*pProductRevisionMinor = (revision_id & 0xF0) >> 4;


    return Status;

}

VL53L0_Error VL53L0_GetDeviceInfo(VL53L0_DEV Dev, VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t revision_id;
    uint8_t Revision;


    Status = VL53L0_check_part_used(Dev, &Revision, pVL53L0_DeviceInfo);

    if (Status == VL53L0_ERROR_NONE) {
        if (Revision == 0) {
            VL53L0_COPYSTRING(pVL53L0_DeviceInfo->Name,
            		VL53L0_STRING_DEVICE_INFO_NAME_TS0);
        } else if ((Revision <= 34) && (Revision != 32)) {
            VL53L0_COPYSTRING(pVL53L0_DeviceInfo->Name,
            		VL53L0_STRING_DEVICE_INFO_NAME_TS1);
        } else if (Revision < 39) {
            VL53L0_COPYSTRING(pVL53L0_DeviceInfo->Name,
            		VL53L0_STRING_DEVICE_INFO_NAME_TS2);
        } else {
            VL53L0_COPYSTRING(pVL53L0_DeviceInfo->Name,
            		VL53L0_STRING_DEVICE_INFO_NAME_ES1);
        }

        VL53L0_COPYSTRING(pVL53L0_DeviceInfo->Type,
        		VL53L0_STRING_DEVICE_INFO_TYPE);

    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_IDENTIFICATION_MODEL_ID,
        		&pVL53L0_DeviceInfo->ProductType);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_IDENTIFICATION_REVISION_ID,
        		&revision_id);
        pVL53L0_DeviceInfo->ProductRevisionMajor = 1;
        pVL53L0_DeviceInfo->ProductRevisionMinor = (revision_id & 0xF0) >> 4;
    }

    return Status;
}

VL53L0_Error VL53L0_GetDeviceErrorStatus(VL53L0_DEV Dev,
		VL53L0_DeviceError* pDeviceErrorStatus)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t RangeStatus;


    Status = VL53L0_RdByte(Dev, VL53L0_REG_RESULT_RANGE_STATUS, &RangeStatus);

    *pDeviceErrorStatus = (VL53L0_DeviceError)((RangeStatus & 0x78) >> 3);

    return Status;
}

#define VL53L0_BUILDCASESTRING(BUFFER, CODE,STRINGVALUE) \
    case CODE: \
    VL53L0_COPYSTRING(BUFFER, STRINGVALUE); \
    break;\

VL53L0_Error VL53L0_GetDeviceErrorString(VL53L0_DeviceError ErrorCode,
		char* pDeviceErrorString)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    switch (ErrorCode)
    {
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_NONE,
            VL53L0_STRING_DEVICEERROR_NONE);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
    		VL53L0_DEVICEERROR_VCSELCONTINUITYTESTFAILURE,
    		VL53L0_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
    		VL53L0_DEVICEERROR_VCSELWATCHDOGTESTFAILURE,
    		VL53L0_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
    		VL53L0_DEVICEERROR_NOVHVVALUEFOUND,
    		VL53L0_STRING_DEVICEERROR_NOVHVVALUEFOUND);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_MSRCNOTARGET,
    		VL53L0_STRING_DEVICEERROR_MSRCNOTARGET);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_SNRCHECK,
    		VL53L0_STRING_DEVICEERROR_SNRCHECK);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
    		VL53L0_DEVICEERROR_RANGEPHASECHECK,
    		VL53L0_STRING_DEVICEERROR_RANGEPHASECHECK);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
    		VL53L0_DEVICEERROR_SIGMATHRESHOLDCHECK,
    		VL53L0_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_TCC,
    		VL53L0_STRING_DEVICEERROR_TCC);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
    		VL53L0_DEVICEERROR_PHASECONSISTENCY,
    		VL53L0_STRING_DEVICEERROR_PHASECONSISTENCY);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_MINCLIP,
    		VL53L0_STRING_DEVICEERROR_MINCLIP);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_RANGECOMPLETE,
    		VL53L0_STRING_DEVICEERROR_RANGECOMPLETE);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_ALGOUNDERFLOW,
    		VL53L0_STRING_DEVICEERROR_ALGOUNDERFLOW);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
            VL53L0_DEVICEERROR_ALGOOVERFLOW,
    		VL53L0_STRING_DEVICEERROR_ALGOOVERFLOW);
    VL53L0_BUILDCASESTRING(pDeviceErrorString,
    		VL53L0_DEVICEERROR_RANGEIGNORETHRESHOLD,
    		VL53L0_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD);
    default:
        VL53L0_COPYSTRING(pDeviceErrorString,
            VL53L0_STRING_UNKNOW_ERROR_CODE);

    }

    return Status;
}

VL53L0_Error VL53L0_GetRangeStatusString(uint8_t RangeStatus,
		char* pRangeStatusString)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    switch (RangeStatus)
    {
    VL53L0_BUILDCASESTRING(pRangeStatusString, 0,
    		VL53L0_STRING_RANGESTATUS_RANGEVALID);
    VL53L0_BUILDCASESTRING(pRangeStatusString, 1,
    		VL53L0_STRING_RANGESTATUS_SIGMA);
    VL53L0_BUILDCASESTRING(pRangeStatusString, 2,
    		VL53L0_STRING_RANGESTATUS_SIGNAL);
    VL53L0_BUILDCASESTRING(pRangeStatusString, 3,
    		VL53L0_STRING_RANGESTATUS_MINRANGE);
    VL53L0_BUILDCASESTRING(pRangeStatusString, 4,
            VL53L0_STRING_RANGESTATUS_PHASE);
    VL53L0_BUILDCASESTRING(pRangeStatusString, 5,
            VL53L0_STRING_RANGESTATUS_HW);
    default: 
        VL53L0_COPYSTRING(pRangeStatusString,
        		VL53L0_STRING_RANGESTATUS_NONE);

    }

    return Status;
}

VL53L0_Error VL53L0_GetPalErrorString(VL53L0_Error PalErrorCode,
		char* pPalErrorString)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    switch (PalErrorCode)
    {
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_NONE,
            VL53L0_STRING_ERROR_NONE);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_CALIBRATION_WARNING,
            VL53L0_STRING_ERROR_CALIBRATION_WARNING);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_MIN_CLIPPED,
            VL53L0_STRING_ERROR_MIN_CLIPPED);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_UNDEFINED,
            VL53L0_STRING_ERROR_UNDEFINED);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_INVALID_PARAMS,
            VL53L0_STRING_ERROR_INVALID_PARAMS);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_NOT_SUPPORTED,
            VL53L0_STRING_ERROR_NOT_SUPPORTED);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_RANGE_ERROR,
            VL53L0_STRING_ERROR_RANGE_ERROR);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_TIME_OUT,
            VL53L0_STRING_ERROR_TIME_OUT);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_MODE_NOT_SUPPORTED,
            VL53L0_STRING_ERROR_MODE_NOT_SUPPORTED);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_BUFFER_TOO_SMALL,
            VL53L0_STRING_ERROR_BUFFER_TOO_SMALL);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_GPIO_NOT_EXISTING,
            VL53L0_STRING_ERROR_GPIO_NOT_EXISTING);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED,
            VL53L0_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_CONTROL_INTERFACE,
            VL53L0_STRING_ERROR_CONTROL_INTERFACE);
    VL53L0_BUILDCASESTRING(pPalErrorString,
    		VL53L0_ERROR_INVALID_COMMAND,
            VL53L0_STRING_ERROR_INVALID_COMMAND);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_DIVISION_BY_ZERO,
            VL53L0_STRING_ERROR_DIVISION_BY_ZERO);
    VL53L0_BUILDCASESTRING(pPalErrorString,
    		VL53L0_ERROR_REF_SPAD_INIT,
            VL53L0_STRING_ERROR_REF_SPAD_INIT);
    VL53L0_BUILDCASESTRING(pPalErrorString,
            VL53L0_ERROR_NOT_IMPLEMENTED,
            VL53L0_STRING_ERROR_NOT_IMPLEMENTED);
    default:
        VL53L0_COPYSTRING(pPalErrorString,
                          VL53L0_STRING_UNKNOW_ERROR_CODE);
    }

    return Status;
}


VL53L0_Error VL53L0_GetPalStateString(VL53L0_State PalStateCode,
		char* pPalStateString)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    switch (PalStateCode)
    {
    VL53L0_BUILDCASESTRING(pPalStateString,
    		VL53L0_STATE_POWERDOWN,
            VL53L0_STRING_STATE_POWERDOWN);
    VL53L0_BUILDCASESTRING(pPalStateString,
    		VL53L0_STATE_WAIT_STATICINIT,
            VL53L0_STRING_STATE_WAIT_STATICINIT);
    VL53L0_BUILDCASESTRING(pPalStateString,
    		VL53L0_STATE_STANDBY,
            VL53L0_STRING_STATE_STANDBY);
    VL53L0_BUILDCASESTRING(pPalStateString,
    		VL53L0_STATE_IDLE,
            VL53L0_STRING_STATE_IDLE);
    VL53L0_BUILDCASESTRING(pPalStateString,
    		VL53L0_STATE_RUNNING,
            VL53L0_STRING_STATE_RUNNING);
    VL53L0_BUILDCASESTRING(pPalStateString,
    		VL53L0_STATE_UNKNOWN,
    		VL53L0_STRING_STATE_UNKNOWN);
    VL53L0_BUILDCASESTRING(pPalStateString,
    		VL53L0_STATE_ERROR,
            VL53L0_STRING_STATE_ERROR);
    default:
        VL53L0_COPYSTRING(pPalStateString,
        		VL53L0_STRING_STATE_UNKNOWN);
    }

    return Status;
}


VL53L0_Error VL53L0_GetPalState(VL53L0_DEV Dev, VL53L0_State* pPalState) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    *pPalState = PALDevDataGet(Dev, PalState);

    return Status;
}

VL53L0_Error VL53L0_SetPowerMode(VL53L0_DEV Dev, VL53L0_PowerModes PowerMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    
    if ((PowerMode != VL53L0_POWERMODE_STANDBY_LEVEL1) &&
        (PowerMode != VL53L0_POWERMODE_IDLE_LEVEL1)) {
        Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
    } else if (PowerMode == VL53L0_POWERMODE_STANDBY_LEVEL1) {
        
        Status = VL53L0_WrByte(Dev, 0x80, 0x00);
        if (Status == VL53L0_ERROR_NONE) {
            
            PALDevDataSet(Dev, PalState, VL53L0_STATE_STANDBY);
            PALDevDataSet(Dev, PowerMode, VL53L0_POWERMODE_STANDBY_LEVEL1);
        }

    } else {
        

        Status = VL53L0_WrByte(Dev, 0x80, 0x00);
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_StaticInit(Dev);
        }
        if (Status == VL53L0_ERROR_NONE) {
            PALDevDataSet(Dev, PowerMode, VL53L0_POWERMODE_IDLE_LEVEL1);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetPowerMode(VL53L0_DEV Dev, VL53L0_PowerModes* pPowerMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte;

    
    Status = VL53L0_RdByte(Dev, 0x80, &Byte);

    if (Status == VL53L0_ERROR_NONE) {
        if (Byte == 1) {
            PALDevDataSet(Dev, PowerMode, VL53L0_POWERMODE_IDLE_LEVEL1);
        } else {
            PALDevDataSet(Dev, PowerMode, VL53L0_POWERMODE_STANDBY_LEVEL1);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_SetOffsetCalibrationDataMicroMeter(VL53L0_DEV Dev,
		int32_t OffsetCalibrationDataMicroMeter)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int32_t cMaxOffsetMicroMeter = 511000;
    int32_t cMinOffsetMicroMeter = -512000;
    int32_t WholeNumber_mm = 0;
    uint32_t Fraction_mm = 0;
    uint32_t OffsetCalibrationDataMilliMeter = 0;


    if(OffsetCalibrationDataMicroMeter > cMaxOffsetMicroMeter)
    {
        OffsetCalibrationDataMicroMeter = cMaxOffsetMicroMeter;
    }
    else if(OffsetCalibrationDataMicroMeter < cMinOffsetMicroMeter)
    {
        OffsetCalibrationDataMicroMeter = cMinOffsetMicroMeter;
    }

    WholeNumber_mm = OffsetCalibrationDataMicroMeter/1000;
    Fraction_mm = OffsetCalibrationDataMicroMeter - (WholeNumber_mm * 1000);

    if(OffsetCalibrationDataMicroMeter >= 0)
    {
        OffsetCalibrationDataMilliMeter
            = (WholeNumber_mm << 2) + (((Fraction_mm * 0x3) + 500)/1000);
    }
    else
    {
        WholeNumber_mm = abs(WholeNumber_mm);
        Fraction_mm = abs(Fraction_mm);
        OffsetCalibrationDataMilliMeter = ((0x003ff - WholeNumber_mm) << 2);
        OffsetCalibrationDataMilliMeter =
            ((0x003ff - WholeNumber_mm) << 2) + ((Fraction_mm * 0x3 + 500)/1000);
    }

    Status = VL53L0_WrWord(Dev, VL53L0_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM,
    		(uint16_t)(OffsetCalibrationDataMilliMeter));

    return Status;
}

VL53L0_Error VL53L0_GetOffsetCalibrationDataMicroMeter(VL53L0_DEV Dev,
		int32_t * pOffsetCalibrationDataMicroMeter)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t RangeOffsetRegister;
    int16_t cMaxOffset = 511;
    int16_t cOffsetRange = 1024;


    Status = VL53L0_RdWord(Dev,
                           VL53L0_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM,
                           &RangeOffsetRegister);

    if (Status == VL53L0_ERROR_NONE) {

        RangeOffsetRegister = ((RangeOffsetRegister & 0x0fff) + 0x02) >> 2;

        
        if(RangeOffsetRegister > cMaxOffset)
        {
            *pOffsetCalibrationDataMicroMeter =
                (int16_t)(RangeOffsetRegister - cOffsetRange) * 1000;
        }
        else
        {
            *pOffsetCalibrationDataMicroMeter =
                (int16_t)RangeOffsetRegister * 1000;
        }
    }

    return Status;
}



VL53L0_API VL53L0_Error VL53L0_SetLinearityCorrectiveGain(VL53L0_DEV Dev,
            int16_t LinearityCorrectiveGain) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    if ((LinearityCorrectiveGain < 0) || (LinearityCorrectiveGain > 1000))
    	Status = VL53L0_ERROR_INVALID_PARAMS;
    else {
    	PALDevDataSet(Dev, LinearityCorrectiveGain, LinearityCorrectiveGain);

        if  (LinearityCorrectiveGain != 1000) {
        	
        	Status = VL53L0_WrWord(Dev,
    					   VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
    					   0);
        }
    }

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_GetLinearityCorrectiveGain(VL53L0_DEV Dev,
            int16_t * pLinearityCorrectiveGain) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    *pLinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

    return Status;
}


VL53L0_Error VL53L0_SetGroupParamHold(VL53L0_DEV Dev, uint8_t GroupParamHold) {
    VL53L0_Error Status = VL53L0_ERROR_NOT_IMPLEMENTED;

    

    return Status;
}

VL53L0_Error VL53L0_GetUpperLimitMilliMeter(VL53L0_DEV Dev, uint16_t* pUpperLimitMilliMeter) {
    VL53L0_Error Status = VL53L0_ERROR_NOT_IMPLEMENTED;

    

    return Status;
}

VL53L0_Error VL53L0_SetDeviceAddress(VL53L0_DEV Dev, uint8_t DeviceAddress) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    Status = VL53L0_WrByte(Dev, VL53L0_REG_I2C_SLAVE_DEVICE_ADDRESS, DeviceAddress / 2);

    return Status;
}

VL53L0_Error VL53L0_apply_offset_adjustment(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    int32_t CorrectedOffsetMicroMeters;
    int32_t CurrentOffsetMicroMeters;

	Status = VL53L0_get_info_from_device(Dev, 7);

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetOffsetCalibrationDataMicroMeter(Dev,
                            &CurrentOffsetMicroMeters);
    }

    
    if (Status == VL53L0_ERROR_NONE) {

        
        PALDevDataSet(Dev, Part2PartOffsetNVMMicroMeter, CurrentOffsetMicroMeters);

        CorrectedOffsetMicroMeters = CurrentOffsetMicroMeters +
                  (int32_t)PALDevDataGet(Dev, Part2PartOffsetAdjustmentNVMMicroMeter);

        Status = VL53L0_SetOffsetCalibrationDataMicroMeter(Dev,
                    CorrectedOffsetMicroMeters);

        
        if (Status == VL53L0_ERROR_NONE) {
            VL53L0_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters,
                    CorrectedOffsetMicroMeters)
        }
    }

    return Status;
}

VL53L0_Error VL53L0_DataInit(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;

    int i;


#ifdef USE_I2C_2V8
    Status = VL53L0_UpdateByte(Dev,
                               VL53L0_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                               0xFE,
                               0x01);
#endif

    
    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0x88, 0x00);

	VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone, 0);

#ifdef USE_IQC_STATION
    if (Status == VL53L0_ERROR_NONE) {
		Status = VL53L0_apply_offset_adjustment(Dev);
    }
#endif

    
	PALDevDataSet(Dev, LinearityCorrectiveGain, 1000);

    VL53L0_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz, 618660);

    
    VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, 0);

    
    Status = VL53L0_GetDeviceParameters(Dev, &CurrentParameters);
    if (Status == VL53L0_ERROR_NONE) {
        
        CurrentParameters.DeviceMode = VL53L0_DEVICEMODE_SINGLE_RANGING;
        CurrentParameters.HistogramMode = VL53L0_HISTOGRAMMODE_DISABLED;
        PALDevDataSet(Dev, CurrentParameters, CurrentParameters);
    }

    
    PALDevDataSet(Dev, SigmaEstRefArray, 100);
    PALDevDataSet(Dev, SigmaEstEffPulseWidth, 900);
    PALDevDataSet(Dev, SigmaEstEffAmbWidth, 500);
    PALDevDataSet(Dev, targetRefRate, 0x0A00); 

    
    PALDevDataSet(Dev, UseInternalTuningSettings, 1);

    
    for (i = 0; i < VL53L0_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
		if (Status == VL53L0_ERROR_NONE) {
				Status |= VL53L0_SetLimitCheckEnable(Dev, i, 1);
			} else {
				break;
		}
    }

    
    if (Status == VL53L0_ERROR_NONE)
    	Status = VL53L0_SetLimitCheckEnable(Dev,
    			VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, 0);

    if (Status == VL53L0_ERROR_NONE)
    	Status = VL53L0_SetLimitCheckEnable(Dev,
    			VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckValue(Dev,
        		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(32 * 65536));
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckValue(Dev,
        		VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
    			(FixPoint1616_t)(25 * 65536 / 100));
    			
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckValue(Dev,
        		VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
    			(FixPoint1616_t)(35 * 65536));
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckValue(Dev,
        		VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
    			(FixPoint1616_t)(0 * 65536));
    }


    if (Status == VL53L0_ERROR_NONE) {

        PALDevDataSet(Dev, SequenceConfig, 0xFF);
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

        PALDevDataSet(Dev, PalState, VL53L0_STATE_WAIT_STATICINIT);
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 0);
    }

    return Status;
}

VL53L0_Error VL53L0_SetTuningSettingBuffer(VL53L0_DEV Dev,
		uint8_t* pTuningSettingBuffer, uint8_t UseInternalTuningSettings)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    if (UseInternalTuningSettings == 1) {
        
        PALDevDataSet(Dev, UseInternalTuningSettings, 1);
    } else {

        
        if (*pTuningSettingBuffer != 0) {
            PALDevDataSet(Dev, pTuningSettingsPointer, pTuningSettingBuffer);
            PALDevDataSet(Dev, UseInternalTuningSettings, 0);

        } else {
            Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetTuningSettingBuffer(VL53L0_DEV Dev,
		uint8_t** ppTuningSettingBuffer, uint8_t* pUseInternalTuningSettings)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    *ppTuningSettingBuffer = PALDevDataGet(Dev, pTuningSettingsPointer);
    *pUseInternalTuningSettings = PALDevDataGet(Dev, UseInternalTuningSettings);

    return Status;
}



VL53L0_Error VL53L0_StaticInit(VL53L0_DEV Dev)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t* pTuningSettingBuffer;
    uint16_t tempword;
    uint8_t  tempbyte;
    uint8_t  UseInternalTuningSettings;
    uint8_t VhvSettings;
    uint8_t PhaseCal;


	Status = VL53L0_get_info_from_device(Dev, 1);

    
    pTuningSettingBuffer = DefaultTuningSettings;

    if (Status == VL53L0_ERROR_NONE) {
		UseInternalTuningSettings = PALDevDataGet(Dev,
				UseInternalTuningSettings);

		if (UseInternalTuningSettings == 0) {
			pTuningSettingBuffer = PALDevDataGet(Dev, pTuningSettingsPointer);
		} else {
			pTuningSettingBuffer = DefaultTuningSettings;
		}
    }

    if (Status == VL53L0_ERROR_NONE)
    	Status = VL53L0_load_tuning_settings(Dev, pTuningSettingBuffer);

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_WrByte(Dev, 0x80, 0x00);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetGpioConfig(Dev, 0, 0,
                VL53L0_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
                VL53L0_INTERRUPTPOLARITY_LOW);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0_RdWord(Dev, 0x84, &tempword);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz,
                VL53L0_FIXPOINT412TOFIXPOINT1616(tempword));
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetDeviceParameters(Dev, &CurrentParameters);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_RANGE_CONFIG, &tempbyte);
        PALDevDataSet(Dev, RangeFractionalEnable, tempbyte);
    }

    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, CurrentParameters, CurrentParameters);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev,
                VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &tempbyte);
        if (Status == VL53L0_ERROR_NONE) {
            PALDevDataSet(Dev, SequenceConfig, tempbyte);
    	}
    }

    if (Status == VL53L0_ERROR_NONE) {
    	Status = VL53L0_SetSequenceStepEnable(Dev,
    			VL53L0_SEQUENCESTEP_TCC, 1);
    }

    if (Status == VL53L0_ERROR_NONE) {
    	Status = VL53L0_SetSequenceStepEnable(Dev,
    		VL53L0_SEQUENCESTEP_MSRC, 0);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_PerformRefCalibration(Dev,
                &VhvSettings, &PhaseCal); 
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, PalState, VL53L0_STATE_IDLE);
    }

    return Status;
}

VL53L0_Error VL53L0_WaitDeviceBooted(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NOT_IMPLEMENTED;

    

    return Status;
}

VL53L0_Error VL53L0_ResetDevice(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte;

    
    Status = VL53L0_WrByte(Dev, VL53L0_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);

    
    if (Status == VL53L0_ERROR_NONE) {
        do {
            Status = VL53L0_RdByte(Dev,
                    VL53L0_REG_IDENTIFICATION_MODEL_ID, &Byte);
        } while (Byte != 0x00);
    }

    
    Status = VL53L0_WrByte(Dev, VL53L0_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);

    
    if (Status == VL53L0_ERROR_NONE) {
        do {
            Status = VL53L0_RdByte(Dev,
                    VL53L0_REG_IDENTIFICATION_MODEL_ID, &Byte);
        } while (Byte == 0x00);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, PalState, VL53L0_STATE_POWERDOWN);
    }

    return Status;
}

VL53L0_Error VL53L0_SetDeviceParameters(VL53L0_DEV Dev,
        const VL53L0_DeviceParameters_t* pDeviceParameters) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int i;
    Status = VL53L0_SetDeviceMode(Dev, pDeviceParameters->DeviceMode);



    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetHistogramMode(Dev, pDeviceParameters->HistogramMode);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetInterMeasurementPeriodMilliSeconds(Dev,
                pDeviceParameters->InterMeasurementPeriodMilliSeconds);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetXTalkCompensationRateMegaCps(Dev,
                pDeviceParameters->XTalkCompensationRateMegaCps);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetOffsetCalibrationDataMicroMeter(Dev,
                pDeviceParameters->RangeOffsetMicroMeters);
    }

    for (i = 0; i < VL53L0_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
        if (Status == VL53L0_ERROR_NONE) {
            Status |= VL53L0_SetLimitCheckEnable(Dev, i,
                    pDeviceParameters->LimitChecksEnable[i]);
        } else {
            break;
        }
        if (Status == VL53L0_ERROR_NONE) {
            Status |= VL53L0_SetLimitCheckValue(Dev, i,
                    pDeviceParameters->LimitChecksValue[i]);
        } else {
            break;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetWrapAroundCheckEnable(Dev,
                pDeviceParameters->WrapAroundCheckEnable);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(Dev,
                pDeviceParameters->MeasurementTimingBudgetMicroSeconds);
    }

    return Status;
}

VL53L0_Error VL53L0_GetDeviceParameters(VL53L0_DEV Dev,
        VL53L0_DeviceParameters_t* pDeviceParameters) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int i;


    Status = VL53L0_GetDeviceMode(Dev, &(pDeviceParameters->DeviceMode));

    if(Status==VL53L0_ERROR_NONE){
       Status = VL53L0_GetHistogramMode(Dev,
               &(pDeviceParameters->HistogramMode));
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetInterMeasurementPeriodMilliSeconds(Dev,
                &(pDeviceParameters->InterMeasurementPeriodMilliSeconds));
    }
    if (Status == VL53L0_ERROR_NONE) {
        pDeviceParameters->XTalkCompensationEnable = 0;
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetXTalkCompensationRateMegaCps(Dev,
                &(pDeviceParameters->XTalkCompensationRateMegaCps));
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetOffsetCalibrationDataMicroMeter(Dev,
                &(pDeviceParameters->RangeOffsetMicroMeters));
    }

    if (Status == VL53L0_ERROR_NONE) {
        for (i = 0; i < VL53L0_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
            if (Status == VL53L0_ERROR_NONE) {
                Status |= VL53L0_GetLimitCheckValue(Dev, i,
                        &(pDeviceParameters->LimitChecksValue[i]));
            } else {
                break;
            }
            if (Status == VL53L0_ERROR_NONE) {
                Status |= VL53L0_GetLimitCheckEnable(Dev, i,
                        &(pDeviceParameters->LimitChecksEnable[i]));
            } else {
                break;
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetWrapAroundCheckEnable(Dev,
                &(pDeviceParameters->WrapAroundCheckEnable));
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetMeasurementTimingBudgetMicroSeconds(Dev,
                &(pDeviceParameters->MeasurementTimingBudgetMicroSeconds));
    }

    return Status;
}

VL53L0_Error VL53L0_SetDeviceMode(VL53L0_DEV Dev,
        VL53L0_DeviceModes DeviceMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;


    switch (DeviceMode) {
    case VL53L0_DEVICEMODE_SINGLE_RANGING:
    case VL53L0_DEVICEMODE_CONTINUOUS_RANGING:
    case VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
    case VL53L0_DEVICEMODE_SINGLE_HISTOGRAM:
    case VL53L0_DEVICEMODE_GPIO_DRIVE:
    case VL53L0_DEVICEMODE_GPIO_OSC:
        
        VL53L0_SETPARAMETERFIELD(Dev, DeviceMode, DeviceMode);
        break;
    default:
        
        Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
    }

    return Status;
}

VL53L0_Error VL53L0_GetDeviceMode(VL53L0_DEV Dev,
        VL53L0_DeviceModes* pDeviceMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;

    VL53L0_GETPARAMETERFIELD(Dev, DeviceMode, *pDeviceMode);

    return Status;
}

VL53L0_Error VL53L0_SetHistogramMode(VL53L0_DEV Dev,
        VL53L0_HistogramModes HistogramMode)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;

    switch (HistogramMode)
    {
    case VL53L0_HISTOGRAMMODE_DISABLED:
    case VL53L0_HISTOGRAMMODE_REFERENCE_ONLY:
    case VL53L0_HISTOGRAMMODE_RETURN_ONLY:
    case VL53L0_HISTOGRAMMODE_BOTH:
        
        VL53L0_SETPARAMETERFIELD(Dev, HistogramMode, HistogramMode);
        break;
    default:
        
        Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
    }

    return Status;
}

VL53L0_Error VL53L0_GetHistogramMode(VL53L0_DEV Dev,
    VL53L0_HistogramModes* pHistogramMode)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;

    VL53L0_GETPARAMETERFIELD(Dev, HistogramMode, *pHistogramMode);

    return Status;
}

VL53L0_Error VL53L0_SetMeasurementTimingBudgetMicroSeconds(VL53L0_DEV Dev,
        uint32_t MeasurementTimingBudgetMicroSeconds)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint32_t FinalRangeTimingBudgetMicroSeconds;
    VL53L0_DeviceParameters_t CurrentParameters;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;
    uint32_t MsrcDccTccTimeoutMicroSeconds  = 2000;
    uint32_t StartOverheadMicroSeconds      = 1320;
    uint32_t EndOverheadMicroSeconds        = 960;
    uint32_t MsrcOverheadMicroSeconds       = 660;
    uint32_t TccOverheadMicroSeconds        = 590;
    uint32_t DssOverheadMicroSeconds        = 690;
    uint32_t PreRangeOverheadMicroSeconds   = 660;
    uint32_t FinalRangeOverheadMicroSeconds = 550;
    uint32_t PreRangeTimeoutMicroSeconds    = 0;
    uint32_t cMinTimingBudgetMicroSeconds   = 26000;
    uint32_t SubTimeout = 0;


    if (MeasurementTimingBudgetMicroSeconds < cMinTimingBudgetMicroSeconds) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    if (Status == VL53L0_ERROR_NONE) {

        FinalRangeTimingBudgetMicroSeconds =
            MeasurementTimingBudgetMicroSeconds -
            (StartOverheadMicroSeconds + EndOverheadMicroSeconds);

        VL53L0_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);

        if(Status == VL53L0_ERROR_NONE &&
           (SchedulerSequenceSteps.TccOn  ||
            SchedulerSequenceSteps.MsrcOn ||
            SchedulerSequenceSteps.DssOn)) {

            
            Status = get_sequence_step_timeout(Dev,
                                               VL53L0_SEQUENCESTEP_MSRC,
                                               &MsrcDccTccTimeoutMicroSeconds);

            

            if (Status == VL53L0_ERROR_NONE) {

                
                if(SchedulerSequenceSteps.TccOn && Status == VL53L0_ERROR_NONE) {

                    SubTimeout =
                        MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;

                    if(SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
                    } else {
                        
                        Status = VL53L0_ERROR_INVALID_PARAMS;
                    }
                }

                
                if(SchedulerSequenceSteps.DssOn && Status == VL53L0_ERROR_NONE) {

                    SubTimeout =
                        2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);

                    if(SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
                    } else {
                        
                        Status = VL53L0_ERROR_INVALID_PARAMS;
                    }
                }
                else if(SchedulerSequenceSteps.MsrcOn && Status == VL53L0_ERROR_NONE) {

                    
                    SubTimeout =
                        MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;

                    if(SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                        FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
                    } else {
                        
                        Status = VL53L0_ERROR_INVALID_PARAMS;
                    }
                }
            }
        }

        if(Status == VL53L0_ERROR_NONE &&
           SchedulerSequenceSteps.PreRangeOn) {

            

            Status = get_sequence_step_timeout(Dev,
                        VL53L0_SEQUENCESTEP_PRE_RANGE,
                        &PreRangeTimeoutMicroSeconds);

            SubTimeout =
                PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;

            if(SubTimeout < FinalRangeTimingBudgetMicroSeconds) {
                FinalRangeTimingBudgetMicroSeconds -= SubTimeout;
            } else {
                
                Status = VL53L0_ERROR_INVALID_PARAMS;
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE &&
        SchedulerSequenceSteps.FinalRangeOn) {

        FinalRangeTimingBudgetMicroSeconds -= FinalRangeOverheadMicroSeconds;

        Status = set_sequence_step_timeout(Dev,
                       VL53L0_SEQUENCESTEP_FINAL_RANGE,
                       FinalRangeTimingBudgetMicroSeconds);

        VL53L0_SETPARAMETERFIELD(Dev,
                       MeasurementTimingBudgetMicroSeconds,
                       MeasurementTimingBudgetMicroSeconds);
    }


    return Status;
}

VL53L0_Error VL53L0_GetMeasurementTimingBudgetMicroSeconds(VL53L0_DEV Dev,
		uint32_t* pMeasurementTimingBudgetMicroSeconds)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t MsrcDccTccTimeoutMicroSeconds  = 2000;
    uint32_t StartOverheadMicroSeconds      = 1910;
    uint32_t EndOverheadMicroSeconds        = 960;
    uint32_t MsrcOverheadMicroSeconds       = 660;
    uint32_t TccOverheadMicroSeconds        = 590;
    uint32_t DssOverheadMicroSeconds        = 690;
    uint32_t PreRangeOverheadMicroSeconds   = 660;
    uint32_t FinalRangeOverheadMicroSeconds = 550;
    uint32_t PreRangeTimeoutMicroSeconds    = 0;


    
    *pMeasurementTimingBudgetMicroSeconds
        = StartOverheadMicroSeconds + EndOverheadMicroSeconds;

    if (Status == VL53L0_ERROR_NONE) {

        VL53L0_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);
    }

    if (Status == VL53L0_ERROR_NONE) {

        if(SchedulerSequenceSteps.TccOn  ||
           SchedulerSequenceSteps.MsrcOn ||
           SchedulerSequenceSteps.DssOn)
        {
            Status = get_sequence_step_timeout(Dev,
                                               VL53L0_SEQUENCESTEP_MSRC,
                                               &MsrcDccTccTimeoutMicroSeconds);
            if (Status == VL53L0_ERROR_NONE) {
                if(SchedulerSequenceSteps.TccOn)
                {
                    *pMeasurementTimingBudgetMicroSeconds +=
                        MsrcDccTccTimeoutMicroSeconds + TccOverheadMicroSeconds;
                }

                if(SchedulerSequenceSteps.DssOn)
                {
                    *pMeasurementTimingBudgetMicroSeconds +=
                        2 * (MsrcDccTccTimeoutMicroSeconds + DssOverheadMicroSeconds);
                }
                else if(SchedulerSequenceSteps.MsrcOn)
                {
                    *pMeasurementTimingBudgetMicroSeconds +=
                        MsrcDccTccTimeoutMicroSeconds + MsrcOverheadMicroSeconds;
                }
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE) {

        if(SchedulerSequenceSteps.PreRangeOn)
        {
            Status = get_sequence_step_timeout(Dev,
                        VL53L0_SEQUENCESTEP_PRE_RANGE,
                        &PreRangeTimeoutMicroSeconds);
            *pMeasurementTimingBudgetMicroSeconds +=
                PreRangeTimeoutMicroSeconds + PreRangeOverheadMicroSeconds;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {

        if(SchedulerSequenceSteps.FinalRangeOn)
        {
            Status = get_sequence_step_timeout(Dev,
                        VL53L0_SEQUENCESTEP_FINAL_RANGE,
                        &FinalRangeTimeoutMicroSeconds);
            *pMeasurementTimingBudgetMicroSeconds +=
                (FinalRangeTimeoutMicroSeconds + FinalRangeOverheadMicroSeconds);
        }
    }

    if (Status == VL53L0_ERROR_NONE) {

            VL53L0_SETPARAMETERFIELD(Dev, MeasurementTimingBudgetMicroSeconds,
            		*pMeasurementTimingBudgetMicroSeconds);
    }
    return Status;
}

VL53L0_Error VL53L0_SetVcselPulsePeriod(VL53L0_DEV Dev,
         VL53L0_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	uint8_t vcsel_period_reg;
    uint8_t MinPreVcselPeriodPCLK = 12;
    uint8_t MaxPreVcselPeriodPCLK = 18;
    uint8_t MinFinalVcselPeriodPCLK = 8;
    uint8_t MaxFinalVcselPeriodPCLK = 14;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint32_t MeasurementTimingBudgetMicroSeconds;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t PreRangeTimeoutMicroSeconds;
    uint32_t MsrcTimeoutMicroSeconds;


    

    if((VCSELPulsePeriodPCLK % 2) != 0) {

        
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }
    else if(VcselPeriodType == VL53L0_VCSEL_PERIOD_PRE_RANGE &&
            (VCSELPulsePeriodPCLK < MinPreVcselPeriodPCLK ||
             VCSELPulsePeriodPCLK > MaxPreVcselPeriodPCLK)) {

        Status = VL53L0_ERROR_INVALID_PARAMS;
    }
    else if(VcselPeriodType == VL53L0_VCSEL_PERIOD_FINAL_RANGE &&
            (VCSELPulsePeriodPCLK < MinFinalVcselPeriodPCLK ||
             VCSELPulsePeriodPCLK > MaxFinalVcselPeriodPCLK)) {

        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    

    if(Status == VL53L0_ERROR_NONE)
    {

        if(VcselPeriodType == VL53L0_VCSEL_PERIOD_PRE_RANGE) {

            
            if(VCSELPulsePeriodPCLK == 12) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
            }
            else if(VCSELPulsePeriodPCLK == 14) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
            }
            else if(VCSELPulsePeriodPCLK == 16) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
            }
            else if(VCSELPulsePeriodPCLK == 18) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
            }
        }
        else if(VcselPeriodType == VL53L0_VCSEL_PERIOD_FINAL_RANGE) {

            if(VCSELPulsePeriodPCLK == 8) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);

                Status |= VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);

                Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_LIM, 0x30);
                Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
            }
            else if(VCSELPulsePeriodPCLK == 10) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);

                Status |= VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);

                Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_LIM, 0x20);
                Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
            }
            else if(VCSELPulsePeriodPCLK == 12) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);

                Status |= VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);

                Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_LIM, 0x20);
                Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
            }
            else if(VCSELPulsePeriodPCLK == 14) {

                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x048);
                Status = VL53L0_WrByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);

                Status |= VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);

                Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
                Status |= VL53L0_WrByte(Dev, VL53L0_REG_ALGO_PHASECAL_LIM, 0x20);
                Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
            }
        }
    }

    

    if(Status == VL53L0_ERROR_NONE)
    {
        vcsel_period_reg = VL53L0_encode_vcsel_period((uint8_t)
    		VCSELPulsePeriodPCLK);

        /* When the VCSEL period for the pre or final range is changed,
         * the corresponding timeout must be read from the device using
         * the current VCSEL period, then the new VCSEL period can be
         * applied. The timeout then must be written back to the device
         * using the new VCSEL period.
         *
         * For the MSRC timeout, the same applies - this timeout being
         * dependant on the pre-range vcsel period.
         */
        switch (VcselPeriodType)
        {
            case VL53L0_VCSEL_PERIOD_PRE_RANGE:
                Status = get_sequence_step_timeout(Dev,
                            VL53L0_SEQUENCESTEP_PRE_RANGE,
                            &PreRangeTimeoutMicroSeconds);

                Status = get_sequence_step_timeout(Dev,
                            VL53L0_SEQUENCESTEP_MSRC,
                            &MsrcTimeoutMicroSeconds);

                if (Status == VL53L0_ERROR_NONE) {

                    Status = VL53L0_WrByte(Dev,
                                VL53L0_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
                                vcsel_period_reg);
                }

                if (Status == VL53L0_ERROR_NONE) {
                    Status = set_sequence_step_timeout(Dev,
                                VL53L0_SEQUENCESTEP_PRE_RANGE,
                                PreRangeTimeoutMicroSeconds);
                }

                if (Status == VL53L0_ERROR_NONE) {
                    Status = set_sequence_step_timeout(Dev,
                                VL53L0_SEQUENCESTEP_MSRC,
                                MsrcTimeoutMicroSeconds);
                }
                break;
            case VL53L0_VCSEL_PERIOD_FINAL_RANGE:
                Status = get_sequence_step_timeout(Dev,
                            VL53L0_SEQUENCESTEP_FINAL_RANGE,
                            &FinalRangeTimeoutMicroSeconds);

                if (Status == VL53L0_ERROR_NONE) {
                    Status = VL53L0_WrByte(Dev,
                                       VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                                       vcsel_period_reg);
                }

                if (Status == VL53L0_ERROR_NONE) {
                    Status = set_sequence_step_timeout(Dev,
                                VL53L0_SEQUENCESTEP_FINAL_RANGE,
                                FinalRangeTimeoutMicroSeconds);
                }
                break;
            default:
                Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_GETPARAMETERFIELD(Dev,
                                 MeasurementTimingBudgetMicroSeconds,
                                 MeasurementTimingBudgetMicroSeconds);

        Status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(Dev,
                                 MeasurementTimingBudgetMicroSeconds);
    }

	return Status;
}

VL53L0_Error VL53L0_GetVcselPulsePeriod(VL53L0_DEV Dev,
         VL53L0_VcselPeriod VcselPeriodType, uint8_t* pVCSELPulsePeriodPCLK)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t vcsel_period_reg;


    switch (VcselPeriodType)
    {
        case VL53L0_VCSEL_PERIOD_PRE_RANGE:
            Status = VL53L0_RdByte(Dev,
            		VL53L0_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
            		&vcsel_period_reg);
            break;
        case VL53L0_VCSEL_PERIOD_FINAL_RANGE:
            Status = VL53L0_RdByte(Dev,
            		VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
            		&vcsel_period_reg);
            break;
        default:
            Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    if (Status == VL53L0_ERROR_NONE) {
        *pVCSELPulsePeriodPCLK = VL53L0_decode_vcsel_period(vcsel_period_reg);
    }

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_SetSequenceStepEnable(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t SequenceConfig = 0;
    uint8_t SequenceConfigNew = 0;
    uint32_t MeasurementTimingBudgetMicroSeconds;
    VL53L0_DeviceParameters_t CurrentParameters;

    Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &SequenceConfig);

    SequenceConfigNew = SequenceConfig;

    if (Status == VL53L0_ERROR_NONE) {
        if(SequenceStepEnabled == 1)
        {
            switch (SequenceStepId)
            {
                case VL53L0_SEQUENCESTEP_TCC :
                    SequenceConfigNew |= 0x10;
                    break;
                case VL53L0_SEQUENCESTEP_DSS :
                    SequenceConfigNew |= 0x28;
                    break;
                case VL53L0_SEQUENCESTEP_MSRC :
                    SequenceConfigNew |= 0x04;
                    break;
                case VL53L0_SEQUENCESTEP_PRE_RANGE :
                    SequenceConfigNew |= 0x40;
                    break;
                case VL53L0_SEQUENCESTEP_FINAL_RANGE :
                    SequenceConfigNew |= 0x80;
                    break;
                default:
                    Status = VL53L0_ERROR_INVALID_PARAMS;
            }
        }
        else
        {
            switch (SequenceStepId)
            {
                case VL53L0_SEQUENCESTEP_TCC :
                    SequenceConfigNew &= 0xef;
                    break;
                case VL53L0_SEQUENCESTEP_DSS :
                    SequenceConfigNew &= 0xd7;
                    break;
                case VL53L0_SEQUENCESTEP_MSRC :
                    SequenceConfigNew &= 0xfb;
                    break;
                case VL53L0_SEQUENCESTEP_PRE_RANGE :
                    SequenceConfigNew &= 0xbf;
                    break;
                case VL53L0_SEQUENCESTEP_FINAL_RANGE :
                    SequenceConfigNew &= 0x7f;
                    break;
                default:
                    Status = VL53L0_ERROR_INVALID_PARAMS;
            }
        }
    }

    if(SequenceConfigNew != SequenceConfig)
    {
        
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, SequenceConfigNew);
        }
        if (Status == VL53L0_ERROR_NONE) {
            PALDevDataSet(Dev, SequenceConfig, SequenceConfigNew);
        }

        
        if (Status == VL53L0_ERROR_NONE) {
            VL53L0_GETPARAMETERFIELD(Dev,
                                     MeasurementTimingBudgetMicroSeconds,
                                     MeasurementTimingBudgetMicroSeconds);

            VL53L0_SetMeasurementTimingBudgetMicroSeconds(Dev,
                                                          MeasurementTimingBudgetMicroSeconds);
        }
    }


    return Status;
}

VL53L0_API VL53L0_Error VL53L0_GetSequenceStepEnable(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, uint8_t* pSequenceStepEnabled)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t SequenceConfig = 0;

    Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &SequenceConfig);

    if (Status == VL53L0_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
                                       SequenceStepId,
                                       SequenceConfig,
                                       pSequenceStepEnabled);
    }

    return Status;
}


VL53L0_API VL53L0_Error VL53L0_GetSequenceStepEnables(VL53L0_DEV Dev,
                    VL53L0_SchedulerSequenceSteps_t *pSchedulerSequenceSteps)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t SequenceConfig = 0;

    Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &SequenceConfig);

    if (Status == VL53L0_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
                                       VL53L0_SEQUENCESTEP_TCC,
                                       SequenceConfig,
                                       &pSchedulerSequenceSteps->TccOn);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
                                       VL53L0_SEQUENCESTEP_DSS,
                                       SequenceConfig,
                                       &pSchedulerSequenceSteps->DssOn);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
                                       VL53L0_SEQUENCESTEP_MSRC,
                                       SequenceConfig,
                                       &pSchedulerSequenceSteps->MsrcOn);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
                                       VL53L0_SEQUENCESTEP_PRE_RANGE,
                                       SequenceConfig,
                                       &pSchedulerSequenceSteps->PreRangeOn);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = sequence_step_enabled(Dev,
                                       VL53L0_SEQUENCESTEP_FINAL_RANGE,
                                       SequenceConfig,
                                       &pSchedulerSequenceSteps->FinalRangeOn);
    }

    return Status;
}

VL53L0_Error sequence_step_enabled(VL53L0_DEV Dev,
                                   VL53L0_SequenceStepId SequenceStepId,
                                   uint8_t SequenceConfig,
                                   uint8_t* pSequenceStepEnabled)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    *pSequenceStepEnabled = 0;

    switch (SequenceStepId)
    {
        case VL53L0_SEQUENCESTEP_TCC :
            *pSequenceStepEnabled = (SequenceConfig & 0x10) >> 4;
            break;
        case VL53L0_SEQUENCESTEP_DSS :
            *pSequenceStepEnabled = (SequenceConfig & 0x08) >> 3;
            break;
        case VL53L0_SEQUENCESTEP_MSRC :
            *pSequenceStepEnabled = (SequenceConfig & 0x04) >> 2;
            break;
        case VL53L0_SEQUENCESTEP_PRE_RANGE :
            *pSequenceStepEnabled = (SequenceConfig & 0x40) >> 6;
            break;
        case VL53L0_SEQUENCESTEP_FINAL_RANGE :
            *pSequenceStepEnabled = (SequenceConfig & 0x80) >> 7;
            break;
        default:
            Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_GetNumberOfSequenceSteps(VL53L0_DEV Dev,
                    uint8_t* pNumberOfSequenceSteps)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    *pNumberOfSequenceSteps = VL53L0_SEQUENCESTEP_NUMBER_OF_CHECKS;

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_GetSequenceStepsInfo(
                     VL53L0_SequenceStepId SequenceStepId,
                     char* pSequenceStepsString)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;

        switch (SequenceStepId)
        {
			VL53L0_BUILDCASESTRING(pSequenceStepsString,
					VL53L0_SEQUENCESTEP_TCC,
					VL53L0_STRING_SEQUENCESTEP_TCC);

			VL53L0_BUILDCASESTRING(pSequenceStepsString,
					VL53L0_SEQUENCESTEP_DSS,
					VL53L0_STRING_SEQUENCESTEP_DSS);

			VL53L0_BUILDCASESTRING(pSequenceStepsString,
					VL53L0_SEQUENCESTEP_MSRC,
					VL53L0_STRING_SEQUENCESTEP_MSRC);

			VL53L0_BUILDCASESTRING(pSequenceStepsString,
					VL53L0_SEQUENCESTEP_PRE_RANGE,
					VL53L0_STRING_SEQUENCESTEP_PRE_RANGE);

			VL53L0_BUILDCASESTRING(pSequenceStepsString,
					VL53L0_SEQUENCESTEP_FINAL_RANGE,
					VL53L0_STRING_SEQUENCESTEP_FINAL_RANGE);

            default:
                Status = VL53L0_ERROR_INVALID_PARAMS;
        }


   return Status;
}

VL53L0_API VL53L0_Error VL53L0_SetSequenceStepTimeout(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs)
{
    VL53L0_Error Status  = VL53L0_ERROR_NONE;
    VL53L0_Error Status1 = VL53L0_ERROR_NONE;
    uint32_t TimeoutMicroSeconds = ((TimeOutMilliSecs * 1000) + 0x8000) >> 16;
    uint32_t MeasurementTimingBudgetMicroSeconds;
    VL53L0_DeviceParameters_t CurrentParameters;
    FixPoint1616_t OldTimeOutMicroSeconds;


    
    Status = get_sequence_step_timeout(Dev, SequenceStepId, &OldTimeOutMicroSeconds);

    if (Status == VL53L0_ERROR_NONE) {
        Status = set_sequence_step_timeout(Dev, SequenceStepId, TimeoutMicroSeconds);
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_GETPARAMETERFIELD(Dev,
                                 MeasurementTimingBudgetMicroSeconds,
                                 MeasurementTimingBudgetMicroSeconds);

        Status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(Dev,
                                 MeasurementTimingBudgetMicroSeconds);

        if (Status != VL53L0_ERROR_NONE) {
            Status1 = set_sequence_step_timeout(Dev,
                                  SequenceStepId,
                                  OldTimeOutMicroSeconds);

            if (Status1 == VL53L0_ERROR_NONE) {
                Status1 = VL53L0_SetMeasurementTimingBudgetMicroSeconds(Dev,
                                  MeasurementTimingBudgetMicroSeconds);
            }
        }
    }


    return Status;
}

VL53L0_API VL53L0_Error VL53L0_GetSequenceStepTimeout(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, FixPoint1616_t *pTimeOutMilliSecs)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint32_t TimeoutMicroSeconds;
    uint32_t WholeNumber_ms = 0;
    uint32_t Fraction_ms = 0;

    Status = get_sequence_step_timeout(Dev, SequenceStepId, &TimeoutMicroSeconds);
    if (Status == VL53L0_ERROR_NONE) {
        WholeNumber_ms = TimeoutMicroSeconds/1000;
        Fraction_ms = TimeoutMicroSeconds - (WholeNumber_ms * 1000);
        *pTimeOutMilliSecs = (WholeNumber_ms <<16) + (((Fraction_ms * 0xffff) + 500)/1000);
    }

    return Status;
}

VL53L0_Error VL53L0_SetInterMeasurementPeriodMilliSeconds(VL53L0_DEV Dev,
		uint32_t InterMeasurementPeriodMilliSeconds)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;


    Status = VL53L0_RdWord(Dev, VL53L0_REG_OSC_CALIBRATE_VAL,
    		&osc_calibrate_val);

    if (Status == VL53L0_ERROR_NONE) {
        if (osc_calibrate_val !=0) {
            IMPeriodMilliSeconds =
            		InterMeasurementPeriodMilliSeconds*osc_calibrate_val;
        } else {
            IMPeriodMilliSeconds = InterMeasurementPeriodMilliSeconds;
        }
        Status = VL53L0_WrDWord(Dev,
        		VL53L0_REG_SYSTEM_INTERMEASUREMENT_PERIOD,
        		IMPeriodMilliSeconds);
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETPARAMETERFIELD(Dev, InterMeasurementPeriodMilliSeconds,
        		InterMeasurementPeriodMilliSeconds);
    }

    return Status;
}

VL53L0_Error VL53L0_GetInterMeasurementPeriodMilliSeconds(VL53L0_DEV Dev,
		uint32_t* pInterMeasurementPeriodMilliSeconds)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;


    Status = VL53L0_RdWord(Dev, VL53L0_REG_OSC_CALIBRATE_VAL,
    		&osc_calibrate_val);

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdDWord(Dev, VL53L0_REG_SYSTEM_INTERMEASUREMENT_PERIOD,
        		&IMPeriodMilliSeconds);
    }

    if (Status == VL53L0_ERROR_NONE) {
        if (osc_calibrate_val !=0) {
            *pInterMeasurementPeriodMilliSeconds = IMPeriodMilliSeconds /
            		osc_calibrate_val;
        }
        VL53L0_SETPARAMETERFIELD(Dev, InterMeasurementPeriodMilliSeconds,
        		*pInterMeasurementPeriodMilliSeconds);
    }

    return Status;
}

VL53L0_Error VL53L0_SetXTalkCompensationEnable(VL53L0_DEV Dev,
		uint8_t XTalkCompensationEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    FixPoint1616_t TempFix1616;
    uint16_t LinearityCorrectiveGain;


    LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

    if ((XTalkCompensationEnable == 0) || (LinearityCorrectiveGain != 1000)) {
        TempFix1616 = 0;
    } else {
        VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
        		TempFix1616);
    }

    
	Status = VL53L0_WrWord(Dev,
					   VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
					   VL53L0_FIXPOINT1616TOFIXPOINT313(TempFix1616));

    if (Status == VL53L0_ERROR_NONE) {
        if (XTalkCompensationEnable == 0) {
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 0);
        } else {
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 1);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetXTalkCompensationEnable(VL53L0_DEV Dev,
              uint8_t* pXTalkCompensationEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;

    VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
    *pXTalkCompensationEnable = Temp8;

    return Status;
}

VL53L0_Error VL53L0_SetXTalkCompensationRateMegaCps(VL53L0_DEV Dev,
              FixPoint1616_t XTalkCompensationRateMegaCps)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;
    uint16_t LinearityCorrectiveGain;
    uint16_t data;

    VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
    LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

    if (Temp8 == 0) { 
        VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
        		XTalkCompensationRateMegaCps);
    } else {
        
    	if (LinearityCorrectiveGain == 1000) {
    	  data = VL53L0_FIXPOINT1616TOFIXPOINT313(XTalkCompensationRateMegaCps);
    	} else {
    	  data = 0;
    	}

		Status = VL53L0_WrWord(Dev,
						   VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
						   data);

        if (Status == VL53L0_ERROR_NONE) {
            VL53L0_SETPARAMETERFIELD(Dev,
                                     XTalkCompensationRateMegaCps,
                                     XTalkCompensationRateMegaCps);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetXTalkCompensationRateMegaCps(VL53L0_DEV Dev,
		FixPoint1616_t* pXTalkCompensationRateMegaCps)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t Value;
    FixPoint1616_t TempFix1616;
    VL53L0_DeviceParameters_t CurrentParameters;


    Status = VL53L0_RdWord(Dev,
    		VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
    		(uint16_t*) &Value);
    if (Status == VL53L0_ERROR_NONE) {
        if (Value == 0) { 
            VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            		TempFix1616);
            *pXTalkCompensationRateMegaCps = TempFix1616;
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 0);
        } else {
            TempFix1616 = VL53L0_FIXPOINT313TOFIXPOINT1616(Value);
            *pXTalkCompensationRateMegaCps = TempFix1616;
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            		TempFix1616);
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 1);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_SetRefCalibration(VL53L0_DEV Dev,
		uint8_t VhvSettings, uint8_t PhaseCal)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    
    if(Status==VL53L0_ERROR_NONE) {
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x00);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
		Status |= VL53L0_WrByte(Dev, 0xCB, VhvSettings);
		Status |= VL53L0_UpdateByte(Dev, 0xEE, 0x80, PhaseCal);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x01);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    }

    return Status;
}

VL53L0_Error VL53L0_GetRefCalibration(VL53L0_DEV Dev,
		uint8_t *pVhvSettings, uint8_t *pPhaseCal)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t VhvSettings = 0;
    uint8_t PhaseCal = 0;


    
    if(Status==VL53L0_ERROR_NONE) {
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x00);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
		Status |= VL53L0_RdByte(Dev, 0xCB, &VhvSettings);
    	Status |= VL53L0_RdByte(Dev, 0xEE, &PhaseCal);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x01);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    }
    
    *pVhvSettings = VhvSettings;
	*pPhaseCal = (uint8_t)(PhaseCal&0xEF);

    return Status;
}



VL53L0_Error VL53L0_GetNumberOfLimitCheck(uint16_t* pNumberOfLimitCheck) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    *pNumberOfLimitCheck = VL53L0_CHECKENABLE_NUMBER_OF_CHECKS;

    return Status;
}

VL53L0_Error VL53L0_GetLimitCheckInfo(VL53L0_DEV Dev, uint16_t LimitCheckId,
		char* pLimitCheckString)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    switch (LimitCheckId)
    {
    VL53L0_BUILDCASESTRING(pLimitCheckString,
    		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
    		VL53L0_STRING_CHECKENABLE_SIGMA_FINAL_RANGE);

    VL53L0_BUILDCASESTRING(pLimitCheckString,
    		VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
    		VL53L0_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE);

    VL53L0_BUILDCASESTRING(pLimitCheckString,
    		VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
    		VL53L0_STRING_CHECKENABLE_SIGNAL_REF_CLIP);

    VL53L0_BUILDCASESTRING(pLimitCheckString,
    		VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
    		VL53L0_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD);

    default:
        VL53L0_COPYSTRING(pLimitCheckString, VL53L0_STRING_UNKNOW_ERROR_CODE);

    }

    return Status;
}


VL53L0_Error VL53L0_GetLimitCheckStatus(VL53L0_DEV Dev,
            uint16_t LimitCheckId, uint8_t* pLimitCheckStatus)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;


    if (LimitCheckId >= VL53L0_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    } else {

        VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
        		LimitCheckId, Temp8);

        *pLimitCheckStatus = Temp8;

    }

    return Status;
}


VL53L0_Error VL53L0_SetLimitCheckEnable(VL53L0_DEV Dev, uint16_t LimitCheckId,
		uint8_t LimitCheckEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    FixPoint1616_t TempFix1616 = 0;
    uint8_t LimitCheckEnableInt;


    if (LimitCheckId >= VL53L0_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    } else {
        if (LimitCheckEnable == 0) {
            TempFix1616 = 0;
            LimitCheckEnableInt = 0;
        } else {
            VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
            		LimitCheckId, TempFix1616);
            
            LimitCheckEnableInt = 1;
        }

        switch (LimitCheckId) {

		case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckEnableInt);

			break;

		case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

			Status = VL53L0_WrWord(Dev,
					VL53L0_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
					VL53L0_FIXPOINT1616TOFIXPOINT97(TempFix1616));

			break;

		case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:

			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
					VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckEnableInt);

			break;

		case VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
					VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
					LimitCheckEnableInt);

			break;

		default:
			Status = VL53L0_ERROR_INVALID_PARAMS;

        }

    }

    if (Status == VL53L0_ERROR_NONE) {
        if (LimitCheckEnable == 0) {
            VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
            		LimitCheckId, 0);
        } else {
            VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
            		LimitCheckId, 1);
    }
    }

    return Status;
}


VL53L0_Error VL53L0_GetLimitCheckEnable(VL53L0_DEV Dev, uint16_t LimitCheckId,
		uint8_t *pLimitCheckEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;


    if (LimitCheckId >= VL53L0_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, Temp8);
    *pLimitCheckEnable = Temp8;

    return Status;
}



VL53L0_Error VL53L0_SetLimitCheckValue(VL53L0_DEV Dev, uint16_t LimitCheckId,
		FixPoint1616_t LimitCheckValue)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;


    VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, Temp8);

    if (Temp8 == 0) { 
        VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId,
        		LimitCheckValue);
    } else {

        switch (LimitCheckId) {

		case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckValue);
			break;

		case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

			Status = VL53L0_WrWord(Dev,
					VL53L0_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
					VL53L0_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));

			break;

		case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:

			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
					VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckValue);

			break;

		case VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
					VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD, LimitCheckValue);

			break;

		default:
			Status = VL53L0_ERROR_INVALID_PARAMS;

    }

    if (Status == VL53L0_ERROR_NONE) {
            VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId,
            		LimitCheckValue);
        }
    }

    return Status;
}


VL53L0_Error VL53L0_GetLimitCheckValue(VL53L0_DEV Dev, uint16_t LimitCheckId,
		FixPoint1616_t *pLimitCheckValue)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t EnableZeroValue = 0;
    uint16_t Temp16;
    FixPoint1616_t TempFix1616;


    switch (LimitCheckId) {

	case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
		
		VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
		Status = VL53L0_RdWord(Dev,
				VL53L0_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
				&Temp16);
		if (Status == VL53L0_ERROR_NONE) {
			TempFix1616 = VL53L0_FIXPOINT97TOFIXPOINT1616(Temp16);
		}

		EnableZeroValue = 1;
		break;

	case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:
		
		VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
		
		VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD, TempFix1616);
		EnableZeroValue = 0;
		break;

	default:
		Status = VL53L0_ERROR_INVALID_PARAMS;

    }

    if (Status == VL53L0_ERROR_NONE) {

        if (EnableZeroValue == 1) {

            if (TempFix1616 == 0) { 
                VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                		LimitCheckId, TempFix1616);
                *pLimitCheckValue = TempFix1616;
                VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                		LimitCheckId, 0);
            } else {
                *pLimitCheckValue = TempFix1616;
                VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                		LimitCheckId, TempFix1616);
                VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                		LimitCheckId, 1);
            }
        } else {
            *pLimitCheckValue = TempFix1616;
        }
    }

    return Status;

}

VL53L0_Error VL53L0_GetLimitCheckCurrent(VL53L0_DEV Dev,
		uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_RangingMeasurementData_t LastRangeDataBuffer;


	if (LimitCheckId >= VL53L0_CHECKENABLE_NUMBER_OF_CHECKS) {
		Status = VL53L0_ERROR_INVALID_PARAMS;
	} else {
		switch (LimitCheckId) {
		case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
			
			*pLimitCheckCurrent = PALDevDataGet(Dev, SigmaEstimate);

			break;

		case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
			
			LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);
			*pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;

			break;

		case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:
			
			*pLimitCheckCurrent = PALDevDataGet(Dev, LastSignalRefMcps);

			break;

		case VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
			
			LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);
			*pLimitCheckCurrent = LastRangeDataBuffer.SignalRateRtnMegaCps;

			break;


		default:
			Status = VL53L0_ERROR_INVALID_PARAMS;
		}
	}

	return Status;

}


VL53L0_Error VL53L0_SetWrapAroundCheckEnable(VL53L0_DEV Dev,
		uint8_t WrapAroundCheckEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte;
    uint8_t WrapAroundCheckEnableInt;
    VL53L0_DeviceParameters_t CurrentParameters;


    Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &Byte);
    if (WrapAroundCheckEnable == 0) {
        
        Byte = Byte & 0x7F;
        WrapAroundCheckEnableInt = 0;
    } else {
        
        Byte = Byte | 0x80;
		WrapAroundCheckEnableInt = 1;
    }

    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, Byte);

    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, SequenceConfig, Byte);
        VL53L0_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable,
        		WrapAroundCheckEnableInt);
    }

    return Status;
}

VL53L0_Error VL53L0_GetWrapAroundCheckEnable(VL53L0_DEV Dev,
		uint8_t* pWrapAroundCheckEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t data;
    VL53L0_DeviceParameters_t CurrentParameters;


    Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &data);
    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, SequenceConfig, data);
        if (data & (0x01 << 7)) {
            *pWrapAroundCheckEnable = 0x01;
        } else {
            *pWrapAroundCheckEnable = 0x00;
        }
    }
    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable,
        		*pWrapAroundCheckEnable);
    }

    return Status;
}


 
VL53L0_Error VL53L0_PerformSingleMeasurement(VL53L0_DEV Dev)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceModes DeviceMode;


    
    Status = VL53L0_GetDeviceMode(Dev, &DeviceMode);

    if (Status     == VL53L0_ERROR_NONE &&
        DeviceMode == VL53L0_DEVICEMODE_SINGLE_RANGING)
    {
        Status = VL53L0_StartMeasurement(Dev);
    }

    if (Status == VL53L0_ERROR_NONE) {
        msleep(20);
        Status = VL53L0_measurement_poll_for_completion(Dev);
    }

    
    if (Status     == VL53L0_ERROR_NONE &&
        DeviceMode == VL53L0_DEVICEMODE_SINGLE_RANGING)
    {
        PALDevDataSet(Dev, PalState, VL53L0_STATE_IDLE);
    }

    return Status;
}

VL53L0_Error VL53L0_PerformSingleHistogramMeasurement(VL53L0_DEV Dev,
		VL53L0_HistogramMeasurementData_t* pHistogramMeasurementData)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceModes DeviceMode;
    VL53L0_HistogramModes HistogramMode = VL53L0_HISTOGRAMMODE_DISABLED;
    uint32_t MeasCount;
    uint32_t Measurements;

    
    Status = VL53L0_GetHistogramMode(Dev, &HistogramMode);

    if (Status == VL53L0_ERROR_NONE)
    {
        if(HistogramMode == VL53L0_HISTOGRAMMODE_BOTH)
        {
            if(pHistogramMeasurementData->BufferSize < VL53L0_HISTOGRAM_BUFFER_SIZE)
            {
                Status = VL53L0_ERROR_BUFFER_TOO_SMALL;
            }
        }
        else
        {
            if(pHistogramMeasurementData->BufferSize < VL53L0_HISTOGRAM_BUFFER_SIZE/2)
            {
                Status = VL53L0_ERROR_BUFFER_TOO_SMALL;
            }
        }
        pHistogramMeasurementData->HistogramType = (uint8_t)HistogramMode;
        pHistogramMeasurementData->ErrorStatus   = VL53L0_DEVICEERROR_NONE;
        pHistogramMeasurementData->FirstBin      = 0;
        pHistogramMeasurementData->NumberOfBins  = 0;
    }

    if (Status == VL53L0_ERROR_NONE)
    {
        
        Status = VL53L0_GetDeviceMode(Dev, &DeviceMode);
    }

    if (Status == VL53L0_ERROR_NONE)
    {
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_HISTOGRAM_BIN, 0x00);
    }

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT, 0x00);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_HISTOGRAM_CONFIG_READOUT_CTRL, 0x01);

    if (Status == VL53L0_ERROR_NONE) {
        Measurements = 3;
        if(HistogramMode == VL53L0_HISTOGRAMMODE_BOTH)
        {
            Measurements = 6;
        }

        if(DeviceMode == VL53L0_DEVICEMODE_SINGLE_HISTOGRAM)
        {
            MeasCount = 0;
            while((MeasCount < Measurements) && (Status == VL53L0_ERROR_NONE))
            {
                Status = VL53L0_start_histogram_measurement(Dev, HistogramMode,
                		MeasCount);

                if (Status == VL53L0_ERROR_NONE)
                    VL53L0_confirm_measurement_start(Dev);

                if (Status == VL53L0_ERROR_NONE)
                    PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);

                if (Status == VL53L0_ERROR_NONE)
                    Status = VL53L0_measurement_poll_for_completion(Dev);

                if (Status == VL53L0_ERROR_NONE)
                {
                    Status = VL53L0_read_histo_measurement(Dev,
								   pHistogramMeasurementData->HistogramData,
								   MeasCount,
								   HistogramMode);
                    if(Status==VL53L0_ERROR_NONE)
                    {
                        if(HistogramMode == VL53L0_HISTOGRAMMODE_BOTH)
                            pHistogramMeasurementData->NumberOfBins+= 2;
                        else
                            pHistogramMeasurementData->NumberOfBins+= 4;

                    }
                }

                if(Status==VL53L0_ERROR_NONE)
                    Status = VL53L0_ClearInterruptMask(Dev, 0);

                MeasCount++;
            }
        } else {
            Status = VL53L0_ERROR_INVALID_COMMAND;
        }
    }


    
    if (Status     == VL53L0_ERROR_NONE)
    {
        pHistogramMeasurementData->NumberOfBins = 12;
        PALDevDataSet(Dev, PalState, VL53L0_STATE_IDLE);
    }


    return Status;
}

VL53L0_Error VL53L0_measurement_poll_for_completion(VL53L0_DEV Dev){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t NewDataReady=0;
    uint32_t LoopNb;


    LoopNb = 0;
    do {
        Status = VL53L0_GetMeasurementDataReady(Dev, &NewDataReady);
        LoopNb++;
    } while((NewDataReady == 0 ) &&
            (Status == VL53L0_ERROR_NONE) &&
            (LoopNb < VL53L0_DEFAULT_MAX_LOOP));


    return Status;
}

VL53L0_Error VL53L0_confirm_measurement_start(VL53L0_DEV Dev){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t NewDataReady=0;
    uint32_t LoopNb;


    LoopNb = 0;
    do {
        Status = VL53L0_GetMeasurementDataReady(Dev, &NewDataReady);
        if ((NewDataReady == 0x01) || Status != VL53L0_ERROR_NONE) {
            break;
        }
        LoopNb = LoopNb + 1;
        VL53L0_PollingDelay(Dev);
    } while (LoopNb < VL53L0_DEFAULT_MAX_LOOP);

    if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP) {
        Status = VL53L0_ERROR_TIME_OUT;
    }


    return Status;
}

VL53L0_Error VL53L0_perform_single_ref_calibration(VL53L0_DEV Dev)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte = 0;
    uint32_t LoopNb;


    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START,
        		VL53L0_REG_SYSRANGE_MODE_START_STOP);
    }

    if (Status == VL53L0_ERROR_NONE) {
        
        LoopNb = 0;
        do {
            if (LoopNb > 0)
                Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSRANGE_START, &Byte);
            LoopNb = LoopNb + 1;
        } while (((Byte & VL53L0_REG_SYSRANGE_MODE_START_STOP) ==
        		VL53L0_REG_SYSRANGE_MODE_START_STOP) &&
                  (Status == VL53L0_ERROR_NONE) &&
                  (LoopNb < VL53L0_DEFAULT_MAX_LOOP));
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_measurement_poll_for_completion(Dev);
    }
    if(Status==VL53L0_ERROR_NONE){
        Status = VL53L0_ClearInterruptMask(Dev, 0);
    }

    return Status;
}

VL53L0_Error VL53L0_PerformRefCalibration(VL53L0_DEV Dev, uint8_t *pVhvSettings,
		uint8_t *pPhaseCal)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t SequenceConfig = 0;
    uint8_t PhaseCal = 0;
    uint8_t VhvSettings = 0;



    SequenceConfig = PALDevDataGet(Dev, SequenceConfig);

    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, 0x03);

    
    PALDevDataSet(Dev, SequenceConfig, 0x01);

    if(Status==VL53L0_ERROR_NONE)
    	Status = VL53L0_perform_single_ref_calibration(Dev);

    
    if(Status==VL53L0_ERROR_NONE) {
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x00);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
		Status |= VL53L0_RdByte(Dev, 0xCB, &VhvSettings);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x01);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    }
    
    *pVhvSettings = VhvSettings;

    
    PALDevDataSet(Dev, SequenceConfig, 0x02);

    if(Status==VL53L0_ERROR_NONE)
    	Status = VL53L0_perform_single_ref_calibration(Dev);

    
    if(Status==VL53L0_ERROR_NONE) {
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x00);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    	Status |= VL53L0_RdByte(Dev, 0xEE, &PhaseCal);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
		Status |= VL53L0_WrByte(Dev, 0x00, 0x01);
		Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    }
    
	*pPhaseCal = (uint8_t)(PhaseCal&0xEF);

    if(Status==VL53L0_ERROR_NONE){
        
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG,
        		SequenceConfig);
		if(Status==VL53L0_ERROR_NONE){
			PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
		}
    }

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_PerformXTalkCalibration(VL53L0_DEV Dev,
            FixPoint1616_t XTalkCalDistance,
            FixPoint1616_t* pXTalkCompensationRateMegaCps) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t sum_ranging = 0;
    uint16_t sum_spads =0;
    FixPoint1616_t sum_signalRate = 0;
    FixPoint1616_t total_count = 0;
    uint8_t xtalk_meas = 0;
    VL53L0_RangingMeasurementData_t RangingMeasurementData;
    FixPoint1616_t xTalkStoredMeanSignalRate;
    FixPoint1616_t xTalkStoredMeanRange;
    FixPoint1616_t xTalkStoredMeanRtnSpads;
    uint32_t signalXTalkTotalPerSpad;
    uint32_t xTalkStoredMeanRtnSpadsAsInt;
    uint32_t xTalkCalDistanceAsInt;
    FixPoint1616_t XTalkCompensationRateMegaCps;

    if (XTalkCalDistance<=0) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetXTalkCompensationEnable(Dev, 0);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckEnable(Dev,
        		VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        sum_ranging = 0;
        sum_spads =0;
        sum_signalRate = 0;
        total_count = 0;
        for(xtalk_meas=0;xtalk_meas<50;xtalk_meas++)
        {
            Status = VL53L0_PerformSingleRangingMeasurement(Dev, &RangingMeasurementData);

            if (Status != VL53L0_ERROR_NONE) {
                break;
            }

            
            if (RangingMeasurementData.RangeStatus == 0) {
                sum_ranging = sum_ranging + RangingMeasurementData.RangeMilliMeter;
                sum_signalRate = sum_signalRate + RangingMeasurementData.SignalRateRtnMegaCps;
                sum_spads = sum_spads + RangingMeasurementData.EffectiveSpadRtnCount/256;
                total_count = total_count + 1;
                pr_info("[Laser] (%d) %d, 0x%x\n", total_count, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
            }
        }

        if (total_count == 0) {
            
            Status = VL53L0_ERROR_RANGE_ERROR;
        }
    }


    if (Status == VL53L0_ERROR_NONE) {
        
        xTalkStoredMeanSignalRate = sum_signalRate / total_count;
        xTalkStoredMeanRange = (FixPoint1616_t)((uint32_t)(sum_ranging<<16) / total_count);
        xTalkStoredMeanRtnSpads = (FixPoint1616_t)((uint32_t)(sum_spads<<16) / total_count);

        xTalkStoredMeanRtnSpadsAsInt = (xTalkStoredMeanRtnSpads + 0x8000) >> 16;

         xTalkCalDistanceAsInt = (XTalkCalDistance + 0x8000) >> 16;

        if(xTalkStoredMeanRtnSpadsAsInt == 0 ||
           xTalkCalDistanceAsInt == 0 ||
           xTalkStoredMeanRange >= XTalkCalDistance)
        {
            XTalkCompensationRateMegaCps = 0;
        }
        else
        {
            xTalkCalDistanceAsInt = (XTalkCalDistance + 0x8000) >> 16;

            signalXTalkTotalPerSpad = (xTalkStoredMeanSignalRate)/xTalkStoredMeanRtnSpadsAsInt;

            signalXTalkTotalPerSpad *= ((1<<16) - (xTalkStoredMeanRange/xTalkCalDistanceAsInt));

            
            XTalkCompensationRateMegaCps = (signalXTalkTotalPerSpad + 0x8000) >> 16;

            if (XTalkCompensationRateMegaCps < 0)
            	XTalkCompensationRateMegaCps = 0;
        }

        *pXTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
        pr_info("[Laser] xTalkStoredMeanRange = %d\n", xTalkStoredMeanRange);

        
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_SetXTalkCompensationEnable(Dev, 1);
        }

        
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_SetXTalkCompensationRateMegaCps(Dev,
            		XTalkCompensationRateMegaCps);
        }

    }

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_PerformOffsetCalibration(VL53L0_DEV Dev,
            FixPoint1616_t CalDistanceMilliMeter,
            int32_t* pOffsetMicroMeter) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t sum_ranging = 0;
    FixPoint1616_t total_count = 0;
    VL53L0_RangingMeasurementData_t RangingMeasurementData;
    FixPoint1616_t StoredMeanRange;
    uint32_t StoredMeanRangeAsInt;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint32_t CalDistanceAsInt_mm;
    uint8_t SequenceStepEnabled;
    int meas = 0;

    if (CalDistanceMilliMeter<=0) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    if (Status == VL53L0_ERROR_NONE) {
    	Status = VL53L0_SetOffsetCalibrationDataMicroMeter(Dev, 0);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
    	Status = VL53L0_GetSequenceStepEnable(Dev,
    			VL53L0_SEQUENCESTEP_TCC, &SequenceStepEnabled);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
    	Status = VL53L0_SetSequenceStepEnable(Dev,
    			VL53L0_SEQUENCESTEP_TCC, 0);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckEnable(Dev,
        		VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        sum_ranging = 0;
        total_count = 0;
        for(meas=0;meas<50;meas++)
        {
            Status = VL53L0_PerformSingleRangingMeasurement(Dev, &RangingMeasurementData);

            if (Status != VL53L0_ERROR_NONE) {
                break;
            }

            
            if (RangingMeasurementData.RangeStatus == 0) {
                sum_ranging = sum_ranging + RangingMeasurementData.RangeMilliMeter;
                total_count = total_count + 1;
            }
        }

        if (total_count == 0) {
            
            Status = VL53L0_ERROR_RANGE_ERROR;
        }
    }


    if (Status == VL53L0_ERROR_NONE) {
        
        StoredMeanRange = (FixPoint1616_t)((uint32_t)(sum_ranging<<16) / total_count);

        StoredMeanRangeAsInt = (StoredMeanRange + 0x8000) >> 16;

         CalDistanceAsInt_mm = (CalDistanceMilliMeter + 0x8000) >> 16;

         *pOffsetMicroMeter = (CalDistanceAsInt_mm - StoredMeanRangeAsInt) * 1000;

        
        if (Status == VL53L0_ERROR_NONE) {
            VL53L0_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters, *pOffsetMicroMeter)
            Status = VL53L0_SetOffsetCalibrationDataMicroMeter(Dev, *pOffsetMicroMeter);
        }

    }

    
    if (Status == VL53L0_ERROR_NONE) {
    	if (SequenceStepEnabled != 0) {
			Status = VL53L0_SetSequenceStepEnable(Dev,
					VL53L0_SEQUENCESTEP_TCC, 1);
    	}
    }

    return Status;
}

VL53L0_Error VL53L0_StartMeasurement(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceModes DeviceMode;
    uint8_t Byte;
    uint32_t LoopNb;

    
    VL53L0_GetDeviceMode(Dev, &DeviceMode);

    switch (DeviceMode)
    {
        case VL53L0_DEVICEMODE_SINGLE_RANGING:
            Status = VL53L0_WrByte(Dev,
                        VL53L0_REG_SYSRANGE_START,
                        VL53L0_REG_SYSRANGE_MODE_SINGLESHOT | VL53L0_REG_SYSRANGE_MODE_START_STOP);
            break;
        case VL53L0_DEVICEMODE_CONTINUOUS_RANGING:
            
            Status = VL53L0_WrByte(Dev,
                        VL53L0_REG_SYSRANGE_START,
                        VL53L0_REG_SYSRANGE_MODE_BACKTOBACK | VL53L0_REG_SYSRANGE_MODE_START_STOP);
            if (Status == VL53L0_ERROR_NONE) {
                
                PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);
            }
            break;
        case VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
            
            Status = VL53L0_WrByte(Dev,
                        VL53L0_REG_SYSRANGE_START,
                        VL53L0_REG_SYSRANGE_MODE_TIMED | VL53L0_REG_SYSRANGE_MODE_START_STOP);
            if (Status == VL53L0_ERROR_NONE) {
                
                PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);
            }
            break;
        default:
            
            Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
    }

    Byte = VL53L0_REG_SYSRANGE_MODE_START_STOP;
    if (Status == VL53L0_ERROR_NONE) {
        
        LoopNb = 0;
        do {
            if (LoopNb > 0)
                Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSRANGE_START, &Byte);
            LoopNb = LoopNb + 1;
        } while (((Byte & VL53L0_REG_SYSRANGE_MODE_START_STOP) == VL53L0_REG_SYSRANGE_MODE_START_STOP) &&
                  (Status == VL53L0_ERROR_NONE) &&
                  (LoopNb < VL53L0_DEFAULT_MAX_LOOP));

        if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP) {
            Status = VL53L0_ERROR_TIME_OUT;
        }
    }

    return Status;
}


VL53L0_Error VL53L0_start_histogram_measurement(VL53L0_DEV Dev,
              VL53L0_HistogramModes histoMode,
              uint32_t count)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t dataByte;


    dataByte = VL53L0_REG_SYSRANGE_MODE_SINGLESHOT | VL53L0_REG_SYSRANGE_MODE_START_STOP;
    if(count == 0)
    {
        
        dataByte |= (1 << 5);
    }

    switch (histoMode)
    {
        case VL53L0_HISTOGRAMMODE_DISABLED:
            
            Status = VL53L0_ERROR_INVALID_COMMAND;
            break;

        case VL53L0_HISTOGRAMMODE_REFERENCE_ONLY:
        case VL53L0_HISTOGRAMMODE_RETURN_ONLY:
        case VL53L0_HISTOGRAMMODE_BOTH:
            dataByte |= (histoMode << 3);
            Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START, dataByte);
            if (Status == VL53L0_ERROR_NONE)
            {
                
                PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);
            }
            break;

        default:
            
            Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
    }

    return Status;
}

VL53L0_Error VL53L0_StopMeasurement(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START, VL53L0_REG_SYSRANGE_MODE_SINGLESHOT);

    if (Status == VL53L0_ERROR_NONE) {
        
        PALDevDataSet(Dev, PalState, VL53L0_STATE_IDLE);
    }

    return Status;
}

VL53L0_Error VL53L0_GetMeasurementDataReady(VL53L0_DEV Dev, uint8_t *pMeasurementDataReady) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t SysRangeStatusRegister;
    uint8_t InterruptConfig;
    uint32_t InterruptMask;

    InterruptConfig = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, Pin0GpioFunctionality);

    if (InterruptConfig == VL53L0_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
    	Status = VL53L0_GetInterruptMaskStatus(Dev, &InterruptMask);
        if (InterruptMask == VL53L0_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
            *pMeasurementDataReady = 1;
        } else {
            *pMeasurementDataReady = 0;
        }
    } else {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_RESULT_RANGE_STATUS, &SysRangeStatusRegister);
        if (Status == VL53L0_ERROR_NONE) {
            if (SysRangeStatusRegister & 0x01) {
                *pMeasurementDataReady = 1;
            } else {
                *pMeasurementDataReady = 0;
            }
        }
    }

    return Status;
}

VL53L0_Error VL53L0_WaitDeviceReadyForNewMeasurement(VL53L0_DEV Dev, uint32_t MaxLoop) {
    VL53L0_Error Status = VL53L0_ERROR_NOT_IMPLEMENTED;

    

    return Status;
}

VL53L0_Error VL53L0_GetRangingMeasurementData(VL53L0_DEV Dev,
              VL53L0_RangingMeasurementData_t* pRangingMeasurementData)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t DeviceRangeStatus;
    uint8_t RangeFractionalEnable;
    uint8_t PalRangeStatus;
    uint8_t XTalkCompensationEnable;
    uint16_t AmbientRate;
    FixPoint1616_t SignalRate;
    uint16_t XTalkCompensationRateMegaCps;
    uint16_t EffectiveSpadRtnCount;
    uint16_t tmpuint16;
    uint16_t XtalkRangeMilliMeter;
    uint16_t LinearityCorrectiveGain;
    uint8_t localBuffer[12];
    VL53L0_RangingMeasurementData_t LastRangeDataBuffer;
    VL53L0_DeviceParameters_t CurrentParameters;

    Status = VL53L0_ReadMulti(Dev, 0x14, localBuffer, 12);

    if (Status == VL53L0_ERROR_NONE) {

        pRangingMeasurementData->ZoneId = 0; 
        pRangingMeasurementData->TimeStamp = 0; 

        tmpuint16 = VL53L0_MAKEUINT16(localBuffer[11], localBuffer[10]);


        pRangingMeasurementData->RangeDMaxMilliMeter = 0;
        pRangingMeasurementData->MeasurementTimeUsec = 0;

        SignalRate = VL53L0_FIXPOINT97TOFIXPOINT1616(VL53L0_MAKEUINT16(localBuffer[7], localBuffer[6]));
        
        pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate;

        AmbientRate = VL53L0_MAKEUINT16(localBuffer[9], localBuffer[8]);
        pRangingMeasurementData->AmbientRateRtnMegaCps = VL53L0_FIXPOINT97TOFIXPOINT1616(AmbientRate);

        EffectiveSpadRtnCount = VL53L0_MAKEUINT16(localBuffer[3], localBuffer[2]);
        pRangingMeasurementData->EffectiveSpadRtnCount = EffectiveSpadRtnCount; 

        DeviceRangeStatus = localBuffer[0];


        
        LinearityCorrectiveGain = PALDevDataGet(Dev, LinearityCorrectiveGain);

        
        RangeFractionalEnable = PALDevDataGet(Dev, RangeFractionalEnable);

        if (LinearityCorrectiveGain != 1000) {

			tmpuint16 = (uint16_t)((LinearityCorrectiveGain*tmpuint16
						+ 500)/1000);

			
			VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
					XTalkCompensationRateMegaCps);
			VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationEnable,
					XTalkCompensationEnable);

			if (XTalkCompensationEnable) {

				if( (SignalRate - ((XTalkCompensationRateMegaCps * EffectiveSpadRtnCount)>>8)) <= 0) {
					if (RangeFractionalEnable)
						XtalkRangeMilliMeter =  8888;
					else
						XtalkRangeMilliMeter =  8888<<2;
				} else {
					XtalkRangeMilliMeter =  (tmpuint16 * SignalRate) / (SignalRate -
							((XTalkCompensationRateMegaCps * EffectiveSpadRtnCount)>>8));
				}

				tmpuint16 = XtalkRangeMilliMeter;
			}

        }

        if (RangeFractionalEnable) {
            pRangingMeasurementData->RangeMilliMeter = (uint16_t)((tmpuint16)>>2);
            pRangingMeasurementData->RangeFractionalPart = (uint8_t)((tmpuint16 & 0x03)<<6);
        } else {
            pRangingMeasurementData->RangeMilliMeter = tmpuint16;
            pRangingMeasurementData->RangeFractionalPart = 0;
        }





        

        Status = VL53L0_get_pal_range_status(Dev,
                                             DeviceRangeStatus,
                                             SignalRate,
                                             EffectiveSpadRtnCount,
                                             pRangingMeasurementData,
                                             &PalRangeStatus);

        if (Status == VL53L0_ERROR_NONE) {
            pRangingMeasurementData->RangeStatus = PalRangeStatus;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        
        LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

        LastRangeDataBuffer.RangeMilliMeter =
        		pRangingMeasurementData->RangeMilliMeter;
        LastRangeDataBuffer.RangeFractionalPart =
        		pRangingMeasurementData->RangeFractionalPart;
        LastRangeDataBuffer.RangeDMaxMilliMeter =
        		pRangingMeasurementData->RangeDMaxMilliMeter;
        LastRangeDataBuffer.MeasurementTimeUsec =
        		pRangingMeasurementData->MeasurementTimeUsec;
        LastRangeDataBuffer.SignalRateRtnMegaCps =
        		pRangingMeasurementData->SignalRateRtnMegaCps;
        LastRangeDataBuffer.AmbientRateRtnMegaCps =
        		pRangingMeasurementData->AmbientRateRtnMegaCps;
        LastRangeDataBuffer.EffectiveSpadRtnCount =
        		pRangingMeasurementData->EffectiveSpadRtnCount;
        LastRangeDataBuffer.RangeStatus =
        		pRangingMeasurementData->RangeStatus;

        PALDevDataSet(Dev, LastRangeMeasure, LastRangeDataBuffer);
    }

    return Status;
}


VL53L0_Error VL53L0_GetMeasurementRefSignal(VL53L0_DEV Dev,
		FixPoint1616_t *pMeasurementRefSignal) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

	*pMeasurementRefSignal = PALDevDataGet(Dev, LastSignalRefMcps);

    return Status;

}



VL53L0_Error VL53L0_GetHistogramMeasurementData(VL53L0_DEV Dev,
              VL53L0_HistogramMeasurementData_t* pHistogramMeasurementData) {
    VL53L0_Error Status = VL53L0_ERROR_NOT_IMPLEMENTED;

    return Status;
}

VL53L0_Error VL53L0_read_histo_measurement(VL53L0_DEV Dev,
              uint32_t *histoData,
              uint32_t offset,
              VL53L0_HistogramModes histoMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t localBuffer[28];
    uint32_t cDataSize  = 4;
    uint32_t offset1;


    Status = VL53L0_WrByte(Dev, 0xFF, VL53L0_REG_RESULT_CORE_PAGE);
    Status = VL53L0_ReadMulti(Dev,
                              (uint8_t)VL53L0_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN,
                              localBuffer,
                              28);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);

    if (Status == VL53L0_ERROR_NONE)
    {
        VL53L0_reverse_bytes(&localBuffer[0], cDataSize);
        VL53L0_reverse_bytes(&localBuffer[4], cDataSize);
        VL53L0_reverse_bytes(&localBuffer[20], cDataSize);
        VL53L0_reverse_bytes(&localBuffer[24], cDataSize);

        offset1 = offset * cDataSize;
        if(histoMode == VL53L0_HISTOGRAMMODE_BOTH)
        {

            memcpy(&histoData[offset1],     &localBuffer[4],  cDataSize); 
            memcpy(&histoData[offset1 + 1], &localBuffer[24], cDataSize); 
            memcpy(&histoData[offset1 + 2], &localBuffer[0],  cDataSize); 
            memcpy(&histoData[offset1 + 3], &localBuffer[20], cDataSize); 

        }
        else
        {

            memcpy(&histoData[offset1],     &localBuffer[24], cDataSize);
            memcpy(&histoData[offset1 + 1], &localBuffer[20], cDataSize);
            memcpy(&histoData[offset1 + 2], &localBuffer[4],  cDataSize);
            memcpy(&histoData[offset1 + 3], &localBuffer[0],  cDataSize);

        }
    }


    return Status;
}


VL53L0_Error VL53L0_PerformSingleRangingMeasurement(VL53L0_DEV Dev,
		VL53L0_RangingMeasurementData_t* pRangingMeasurementData)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    Status = VL53L0_SetDeviceMode(Dev, VL53L0_DEVICEMODE_SINGLE_RANGING);

    if (Status == VL53L0_ERROR_NONE) {
    	Status = VL53L0_PerformSingleMeasurement(Dev);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetRangingMeasurementData(Dev, pRangingMeasurementData);
    }

    if(Status==VL53L0_ERROR_NONE){
        Status = VL53L0_ClearInterruptMask(Dev, 0);
    }

    return Status;
}

VL53L0_Error VL53L0_SetNumberOfROIZones(VL53L0_DEV Dev,
		uint8_t NumberOfROIZones)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    if (NumberOfROIZones != 1){
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    return Status;
}

VL53L0_Error VL53L0_GetNumberOfROIZones(VL53L0_DEV Dev,
		uint8_t* pNumberOfROIZones)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    *pNumberOfROIZones = 1;

    return Status;
}

VL53L0_Error VL53L0_GetMaxNumberOfROIZones(VL53L0_DEV Dev,
		uint8_t* pMaxNumberOfROIZones)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;


    *pMaxNumberOfROIZones = 1;

    return Status;
}



VL53L0_Error VL53L0_SetGpioConfig(VL53L0_DEV Dev, uint8_t Pin,
		VL53L0_DeviceModes DeviceMode,
        VL53L0_GpioFunctionality Functionality,
		VL53L0_InterruptPolarity Polarity)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    uint8_t data;


    if (Pin != 0) {
        Status = VL53L0_ERROR_GPIO_NOT_EXISTING;
    } else if (DeviceMode == VL53L0_DEVICEMODE_GPIO_DRIVE) {
        if (Polarity == VL53L0_INTERRUPTPOLARITY_LOW) {
            data = 0x10;
        } else {
            data = 1;
        }
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_GPIO_HV_MUX_ACTIVE_HIGH, data);

    } else if (DeviceMode == VL53L0_DEVICEMODE_GPIO_OSC) {

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
    	Status |= VL53L0_WrByte(Dev, 0x00, 0x00);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
    	Status |= VL53L0_WrByte(Dev, 0x80, 0x01);
    	Status |= VL53L0_WrByte(Dev, 0x85, 0x02);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x04);
    	Status |= VL53L0_WrByte(Dev, 0xcd, 0x00);
    	Status |= VL53L0_WrByte(Dev, 0xcc, 0x11);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x07);
    	Status |= VL53L0_WrByte(Dev, 0xbe, 0x00);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x06);
    	Status |= VL53L0_WrByte(Dev, 0xcc, 0x09);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
    	Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
    	Status |= VL53L0_WrByte(Dev, 0x00, 0x00);

    } else {

		if (Status == VL53L0_ERROR_NONE) {
			switch(Functionality){
				case VL53L0_GPIOFUNCTIONALITY_OFF:
					data = 0x00;
					break;
				case VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
					data = 0x01;
					break;
				case VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
					data = 0x02;
					break;
				case VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
					data = 0x03;
					break;
				case VL53L0_GPIOFUNCTIONALITY_NEW_MEASURE_READY:
					data = 0x04;
					break;
				default:
					Status = VL53L0_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
			}
		}

		if(Status==VL53L0_ERROR_NONE)
			Status = VL53L0_WrByte(Dev,
					VL53L0_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, data);


		if(Status==VL53L0_ERROR_NONE){
			if (Polarity == VL53L0_INTERRUPTPOLARITY_LOW) {
				data = 0;
			} else {
				data = (uint8_t)(1<<4);
			}
			Status = VL53L0_UpdateByte(Dev,
					VL53L0_REG_GPIO_HV_MUX_ACTIVE_HIGH, 0xEF, data);
		}

		if(Status==VL53L0_ERROR_NONE)
			VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
					Pin0GpioFunctionality, Functionality);


		if(Status==VL53L0_ERROR_NONE)
			Status = VL53L0_ClearInterruptMask(Dev, 0);

    }

    return Status;
}

VL53L0_Error VL53L0_GetGpioConfig(VL53L0_DEV Dev, uint8_t Pin,
		VL53L0_DeviceModes* pDeviceMode,
		VL53L0_GpioFunctionality* pFunctionality,
		VL53L0_InterruptPolarity* pPolarity)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    VL53L0_GpioFunctionality GpioFunctionality;
    uint8_t data;


    

    Status = VL53L0_GetDeviceMode(Dev, pDeviceMode);

    if (Status == VL53L0_ERROR_NONE) {
		if(Pin != 0){
			Status = VL53L0_ERROR_GPIO_NOT_EXISTING;
		} else {
			Status = VL53L0_RdByte(Dev,
					VL53L0_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, &data);
		}
	}

    if (Status == VL53L0_ERROR_NONE) {
        switch(data&0x07){
            case 0x00:
                GpioFunctionality = VL53L0_GPIOFUNCTIONALITY_OFF;
                break;
            case 0x01:
                GpioFunctionality =
                		VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW;
                break;
            case 0x02:
                GpioFunctionality =
                		VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH;
                break;
            case 0x03:
                GpioFunctionality =
                		VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT;
                break;
            case 0x04:
                GpioFunctionality = VL53L0_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
                break;
            default:
                Status = VL53L0_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_GPIO_HV_MUX_ACTIVE_HIGH, &data);
    }

    if (Status == VL53L0_ERROR_NONE) {
        if ((data & (uint8_t)(1<<4)) == 0) {
            *pPolarity = VL53L0_INTERRUPTPOLARITY_LOW;
        } else {
            *pPolarity = VL53L0_INTERRUPTPOLARITY_HIGH;
        }
    }

    if(Status==VL53L0_ERROR_NONE){
        *pFunctionality = GpioFunctionality;
        VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
        		Pin0GpioFunctionality, GpioFunctionality);
    }

    return Status;
}

VL53L0_Error VL53L0_SetInterruptThresholds(VL53L0_DEV Dev,
        VL53L0_DeviceModes DeviceMode, FixPoint1616_t ThresholdLow,
        FixPoint1616_t ThresholdHigh) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t Threshold16;

    
    
    Threshold16 = (uint16_t)((ThresholdLow>>17) & 0x00fff);
    Status = VL53L0_WrWord(Dev, VL53L0_REG_SYSTEM_THRESH_LOW, Threshold16);

    if(Status==VL53L0_ERROR_NONE){
        
        Threshold16 = (uint16_t)((ThresholdHigh>>17) & 0x00fff);
        Status = VL53L0_WrWord(Dev, VL53L0_REG_SYSTEM_THRESH_HIGH, Threshold16);
    }

    return Status;
}

VL53L0_Error VL53L0_GetInterruptThresholds(VL53L0_DEV Dev,
    VL53L0_DeviceModes DeviceMode, FixPoint1616_t* pThresholdLow,
    FixPoint1616_t* pThresholdHigh) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t Threshold16;

    

    Status = VL53L0_RdWord(Dev, VL53L0_REG_SYSTEM_THRESH_LOW, &Threshold16);
    
    *pThresholdLow = (FixPoint1616_t)((0x00fff & Threshold16)<<17);

    if(Status==VL53L0_ERROR_NONE){
       Status = VL53L0_RdWord(Dev, VL53L0_REG_SYSTEM_THRESH_HIGH, &Threshold16);
       
       *pThresholdHigh = (FixPoint1616_t)((0x00fff & Threshold16)<<17);
    }

    return Status;
}

VL53L0_Error VL53L0_ClearInterruptMask(VL53L0_DEV Dev, uint32_t InterruptMask) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t LoopCount;
    uint8_t Byte;

    
    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    LoopCount = 0;
    do {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_RESULT_INTERRUPT_STATUS, &Byte);
        LoopCount++;
    } while (((Byte & 0x07) != 0x00) && (LoopCount < 8) && (Status == VL53L0_ERROR_NONE));
    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_INTERRUPT_CLEAR, 0x00);

    return Status;
}

VL53L0_Error VL53L0_GetInterruptMaskStatus(VL53L0_DEV Dev,
        uint32_t *pInterruptMaskStatus)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte;

    Status = VL53L0_RdByte(Dev, VL53L0_REG_RESULT_INTERRUPT_STATUS, &Byte);
    *pInterruptMaskStatus = Byte & 0x07;

    if (Byte & 0x18){
        Status = VL53L0_ERROR_RANGE_ERROR;
    }

    return Status;
}

VL53L0_Error VL53L0_EnableInterruptMask(VL53L0_DEV Dev,
        uint32_t InterruptMask)
{
    VL53L0_Error Status = VL53L0_ERROR_NOT_IMPLEMENTED;

    

    return Status;
}



VL53L0_Error VL53L0_SetSpadAmbientDamperThreshold(VL53L0_DEV Dev,
        uint16_t SpadAmbientDamperThreshold)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status = VL53L0_WrWord(Dev, 0x40, SpadAmbientDamperThreshold);
    VL53L0_WrByte(Dev, 0xFF, 0x00);

    return Status;
}

VL53L0_Error VL53L0_GetSpadAmbientDamperThreshold(VL53L0_DEV Dev,
        uint16_t* pSpadAmbientDamperThreshold)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status = VL53L0_RdWord(Dev, 0x40, pSpadAmbientDamperThreshold);
    VL53L0_WrByte(Dev, 0xFF, 0x00);

    return Status;
}

VL53L0_Error VL53L0_SetSpadAmbientDamperFactor(VL53L0_DEV Dev,
        uint16_t SpadAmbientDamperFactor)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte;

    Byte = (uint8_t) (SpadAmbientDamperFactor & 0x00FF);

    VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status = VL53L0_WrByte(Dev, 0x42, Byte);
    VL53L0_WrByte(Dev, 0xFF, 0x00);

    return Status;
}

VL53L0_Error VL53L0_GetSpadAmbientDamperFactor(VL53L0_DEV Dev,
        uint16_t* pSpadAmbientDamperFactor)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte;

    VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status = VL53L0_RdByte(Dev, 0x42, &Byte);
    VL53L0_WrByte(Dev, 0xFF, 0x00);
    *pSpadAmbientDamperFactor = (uint16_t) Byte;

    return Status;
}



uint32_t VL53L0_calc_macro_period_ps(VL53L0_DEV Dev, uint8_t vcsel_period_pclks);
uint16_t VL53L0_encode_timeout(uint32_t timeout_macro_clks);
uint32_t VL53L0_decode_timeout(uint16_t encoded_timeout);

uint32_t VL53L0_calc_timeout_mclks(VL53L0_DEV Dev,
          uint32_t timeout_period_us,
          uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t timeout_period_mclks = 0;

    macro_period_ps = VL53L0_calc_macro_period_ps(Dev, vcsel_period_pclks);
    macro_period_ns = macro_period_ps / 1000;

    timeout_period_mclks =
        (uint32_t) (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);

    return timeout_period_mclks;
}

uint32_t VL53L0_calc_timeout_us(VL53L0_DEV Dev,
          uint16_t timeout_period_mclks,
          uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t actual_timeout_period_us = 0;

    macro_period_ps = VL53L0_calc_macro_period_ps(Dev, vcsel_period_pclks);
    macro_period_ns = macro_period_ps / 1000;

    actual_timeout_period_us =
        ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;

    return actual_timeout_period_us;
}

uint32_t VL53L0_calc_macro_period_ps(VL53L0_DEV Dev, uint8_t vcsel_period_pclks) {
    uint64_t PLL_period_ps;
    
    uint32_t macro_period_vclks;
    uint32_t macro_period_ps;



    PLL_period_ps = 1655;

    macro_period_vclks = 2304;
    macro_period_ps =
        (uint32_t)(macro_period_vclks * vcsel_period_pclks * PLL_period_ps);

    return macro_period_ps;
}

uint8_t VL53L0_decode_vcsel_period(uint8_t vcsel_period_reg) {


    uint8_t vcsel_period_pclks = 0;

    vcsel_period_pclks = (vcsel_period_reg + 1) << 1;

    return vcsel_period_pclks;
}

uint8_t VL53L0_encode_vcsel_period(uint8_t vcsel_period_pclks) {


    uint8_t vcsel_period_reg = 0;

    vcsel_period_reg = (vcsel_period_pclks >> 1) - 1;

    return vcsel_period_reg;
}

uint16_t VL53L0_encode_timeout(uint32_t timeout_macro_clks) {

    uint16_t encoded_timeout = 0;
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_macro_clks > 0) {
        ls_byte = timeout_macro_clks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte = ls_byte >> 1;
            ms_byte++;
        }

        encoded_timeout = (ms_byte << 8) + (uint16_t) (ls_byte & 0x000000FF);

    }

    return encoded_timeout;

}

uint32_t VL53L0_decode_timeout(uint16_t encoded_timeout) {

    uint32_t timeout_macro_clks = 0;

    timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF) << (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

    return timeout_macro_clks;

}


VL53L0_Error VL53L0_check_part_used(VL53L0_DEV Dev,
		uint8_t* Revision,
        VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t ModuleIdInt;
    char *ProductId_tmp;


    Status = VL53L0_get_info_from_device(Dev, 2);

    if (Status == VL53L0_ERROR_NONE) {
		ModuleIdInt = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, ModuleId);

        if (ModuleIdInt == 0) {
            *Revision = 0;
            VL53L0_COPYSTRING(pVL53L0_DeviceInfo->ProductId, "");
        } else {
            *Revision = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, Revision);
        	ProductId_tmp = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, ProductId);
        	VL53L0_COPYSTRING(pVL53L0_DeviceInfo->ProductId, ProductId_tmp);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_get_info_from_device(VL53L0_DEV Dev, uint8_t option)
{

	VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t byte;
    uint32_t TmpDWord;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    uint8_t ModuleId;
    uint8_t Revision;
    uint8_t ReferenceSpadCount;
    uint8_t ReferenceSpadType;
    uint32_t PartUIDUpper = 0;
    uint32_t PartUIDLower = 0;
    uint32_t OffsetFixed1104_mm = 0;
    int16_t OffsetMicroMeters = 0;
    uint32_t DistMeasTgtFixed1104_mm = 400 << 4;
    uint32_t DistMeasFixed1104_400_mm = 0;
    char ProductId[19];
    char *ProductId_tmp;
    uint8_t ReadDataFromDeviceDone;


    ReadDataFromDeviceDone = VL53L0_GETDEVICESPECIFICPARAMETER(Dev,
    		ReadDataFromDeviceDone);

    if (ReadDataFromDeviceDone == 0) {

        Status |= VL53L0_WrByte(Dev, 0x80, 0x01);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0_WrByte(Dev, 0x00, 0x00);

        Status |= VL53L0_WrByte(Dev, 0xFF, 0x06);
        Status |= VL53L0_RdByte(Dev, 0x83, &byte);
        Status |= VL53L0_WrByte(Dev, 0x83, byte|4);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x07);
        Status |= VL53L0_WrByte(Dev, 0x81, 0x01);

        Status |= VL53L0_PollingDelay(Dev);

        Status |= VL53L0_WrByte(Dev, 0x80, 0x01);

        if ((option&1) == 1) {
			Status |= VL53L0_WrByte(Dev, 0x94, 0x6b);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

			ReferenceSpadCount = (uint8_t)((TmpDWord >> 8) & 0x07f);
			ReferenceSpadType  = (uint8_t)((TmpDWord >> 15) & 0x01);
        }

        if ((option&2) == 2) {

			Status |= VL53L0_WrByte(Dev, 0x94, 0x02);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdByte(Dev, 0x90, &ModuleId);

			Status |= VL53L0_WrByte(Dev, 0x94, 0x7B);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdByte(Dev, 0x90, &Revision);

			Status |= VL53L0_WrByte(Dev, 0x94, 0x77);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

			ProductId[0] = (char)((TmpDWord >> 25) & 0x07f);
			ProductId[1] = (char)((TmpDWord >> 18) & 0x07f);
			ProductId[2] = (char)((TmpDWord >> 11) & 0x07f);
			ProductId[3] = (char)((TmpDWord >> 4) & 0x07f);

			byte = (uint8_t)((TmpDWord & 0x00f) << 3);

			Status |= VL53L0_WrByte(Dev, 0x94, 0x78);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

			ProductId[4] = (char)(byte +
					((TmpDWord >> 29) & 0x07f));
			ProductId[5] = (char)((TmpDWord >> 22) & 0x07f);
			ProductId[6] = (char)((TmpDWord >> 15) & 0x07f);
			ProductId[7] = (char)((TmpDWord >> 8) & 0x07f);
			ProductId[8] = (char)((TmpDWord >> 1) & 0x07f);

			byte = (uint8_t)((TmpDWord & 0x001) << 6);

			Status |= VL53L0_WrByte(Dev, 0x94, 0x79);

			Status |= VL53L0_device_read_strobe(Dev);

			Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

			ProductId[9] = (char)(byte +
					((TmpDWord >> 26) & 0x07f));
			ProductId[10] = (char)((TmpDWord >> 19) & 0x07f);
			ProductId[11] = (char)((TmpDWord >> 12) & 0x07f);
			ProductId[12] = (char)((TmpDWord >> 5) & 0x07f);

			byte = (uint8_t)((TmpDWord & 0x01f) << 2);

			Status |= VL53L0_WrByte(Dev, 0x94, 0x7A);

			Status |= VL53L0_device_read_strobe(Dev);

			Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

			ProductId[13] = (char)(byte +
					((TmpDWord >> 30) & 0x07f));
			ProductId[14] = (char)((TmpDWord >> 23) & 0x07f);
			ProductId[15] = (char)((TmpDWord >> 16) & 0x07f);
			ProductId[16] = (char)((TmpDWord >> 9) & 0x07f);
			ProductId[17] = (char)((TmpDWord >> 2) & 0x07f);
			ProductId[18] = '\0';

        }

        if ((option&4) == 4) {

			Status |= VL53L0_WrByte(Dev, 0x94, 0x7B);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdDWord(Dev, 0x90, &PartUIDUpper);

			Status |= VL53L0_WrByte(Dev, 0x94, 0x7C);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdDWord(Dev, 0x90, &PartUIDLower);

			Status |= VL53L0_WrByte(Dev, 0x94, 0x75);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

			DistMeasFixed1104_400_mm = (TmpDWord & 0x0000000ff) << 8;

			Status |= VL53L0_WrByte(Dev, 0x94, 0x76);
			Status |= VL53L0_device_read_strobe(Dev);
			Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

			DistMeasFixed1104_400_mm |= ((TmpDWord & 0xff000000) >> 24);
        }

        Status |= VL53L0_WrByte(Dev, 0x81, 0x00);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x06);
        Status |= VL53L0_RdByte(Dev, 0x83, &byte);
        Status |= VL53L0_WrByte(Dev, 0x83, byte&0xfb);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0_WrByte(Dev, 0x00, 0x01);

        Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
        Status |= VL53L0_WrByte(Dev, 0x80, 0x00);

        if (Status == VL53L0_ERROR_NONE) {
        	
            if ((option&1) == 1) {
				VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
						ReferenceSpadCount, ReferenceSpadCount);

				VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
						ReferenceSpadType, ReferenceSpadType);
            }

            if ((option&2) == 2) {
				VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
						ModuleId, ModuleId);

				VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
						Revision, Revision);

				ProductId_tmp = VL53L0_GETDEVICESPECIFICPARAMETER(Dev,
						ProductId);
				VL53L0_COPYSTRING(ProductId_tmp, ProductId);
            }

            if ((option&4) == 4) {
				VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
							PartUIDUpper, PartUIDUpper);

				VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
							PartUIDLower, PartUIDLower);

				OffsetMicroMeters = 0;
				if(DistMeasFixed1104_400_mm != 0) {
						OffsetFixed1104_mm =
							DistMeasFixed1104_400_mm - DistMeasTgtFixed1104_mm;
						OffsetMicroMeters = (OffsetFixed1104_mm * 1000) >> 4;
						OffsetMicroMeters *= -1;
				}

				PALDevDataSet(Dev, Part2PartOffsetAdjustmentNVMMicroMeter, OffsetMicroMeters);
            }
            byte = (uint8_t)(ReadDataFromDeviceDone|option);
            VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReadDataFromDeviceDone,
            		byte);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_device_read_strobe(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t strobe;
    uint32_t LoopNb;

    Status |= VL53L0_WrByte(Dev, 0x83, 0x00);

    if (Status == VL53L0_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status =VL53L0_RdByte(Dev, 0x83, &strobe);
            if ((strobe != 0x00) || Status != VL53L0_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
        } while (LoopNb < VL53L0_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP) {
            Status = VL53L0_ERROR_TIME_OUT;
        }
    }

    Status |= VL53L0_WrByte(Dev, 0x83, 0x01);

    return Status;

}


uint32_t VL53L0_isqrt(uint32_t num)
{


    uint32_t  res = 0;
    uint32_t  bit = 1 << 30; 

     
    while (bit > num)
    {
        bit >>= 2;
    }

    while (bit != 0)
    {
        if (num >= res + bit)
        {
            num -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}


uint32_t VL53L0_quadrature_sum(uint32_t a,
                                uint32_t b)
{
    uint32_t  res = 0;

    if( a > 65535 || b > 65535)
    {
        res = 65535;
    }
    else
    {
        res = VL53L0_isqrt(a*a + b*b);
    }

    return res;
}



VL53L0_Error VL53L0_get_jmp_vcsel_ambient_rate(VL53L0_DEV Dev,
                                               uint32_t *pAmbient_rate_kcps,
                                               uint32_t *pVcsel_rate_kcps,
                                               uint32_t *pSignalTotalEventsRtn){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t encodedTimeOut;

    uint32_t    total_periods_elapsed_rtn__macrop  = 0;
    uint32_t    result_core__total_periods_elapsed_rtn  = 0;
    uint32_t    final_range_config__timeout__macrop = 0;
    uint32_t    result_core__ambient_window_events_rtn = 0;
    uint32_t    result_core__signal_total_events_rtn = 0;
    uint8_t     last_woi_period;
    uint8_t     pre_range_config__vcsel_period;
    uint8_t     final_range_config__vcsel_period;
    uint8_t     global_config__vcsel_width;

    uint32_t    ambient_duration_us = 0;
    uint32_t    vcsel_duration_us = 0;

    uint32_t    pll_period_us  = 1655; 


    
    Status = VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0_RdDWord(Dev, 0xc0, &result_core__signal_total_events_rtn);
    Status |= VL53L0_RdDWord(Dev, 0xC8, &result_core__total_periods_elapsed_rtn);
    Status |= VL53L0_RdDWord(Dev, 0xbc, &result_core__ambient_window_events_rtn);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);


    if (Status == VL53L0_ERROR_NONE) {
        result_core__total_periods_elapsed_rtn =
            (int32_t)(result_core__total_periods_elapsed_rtn & 0x00ffffff);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdWord(Dev,
                               VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                               &encodedTimeOut);
        if (Status == VL53L0_ERROR_NONE) {
            final_range_config__timeout__macrop = VL53L0_decode_timeout(encodedTimeOut) - 1;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdWord(Dev,
                               VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                               &encodedTimeOut);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetVcselPulsePeriod(Dev,
                    VL53L0_VCSEL_PERIOD_FINAL_RANGE,
                    &final_range_config__vcsel_period);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetVcselPulsePeriod(Dev,
                    VL53L0_VCSEL_PERIOD_PRE_RANGE,
                    &pre_range_config__vcsel_period);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, 0x32, &global_config__vcsel_width);
    }

    if (Status == VL53L0_ERROR_NONE)
    {
        total_periods_elapsed_rtn__macrop =
        		result_core__total_periods_elapsed_rtn + 1;


        if (result_core__total_periods_elapsed_rtn ==
        		final_range_config__timeout__macrop)
        {
           last_woi_period = final_range_config__vcsel_period;
        }
        else
        {
           last_woi_period = pre_range_config__vcsel_period;
        }

        
        ambient_duration_us = last_woi_period *
        		total_periods_elapsed_rtn__macrop * pll_period_us;
        ambient_duration_us = ambient_duration_us / 1000;

        if (ambient_duration_us != 0) {
            *pAmbient_rate_kcps = ((1 << 15) *
            		result_core__ambient_window_events_rtn) /
            				ambient_duration_us;
        } else {
            Status = VL53L0_ERROR_DIVISION_BY_ZERO;
        }

        if (Status == VL53L0_ERROR_NONE)
        {

            
            vcsel_duration_us = (10*global_config__vcsel_width + 4) *
            		total_periods_elapsed_rtn__macrop * pll_period_us ;
            vcsel_duration_us = vcsel_duration_us / 10000 ;


            if (vcsel_duration_us != 0) {
                *pVcsel_rate_kcps = ((1 << 13) *
                		result_core__signal_total_events_rtn) /
                				vcsel_duration_us;
                *pSignalTotalEventsRtn = result_core__signal_total_events_rtn;
            } else {
                Status = VL53L0_ERROR_DIVISION_BY_ZERO;
            }

        }
    }

    return Status;

}

VL53L0_Error VL53L0_calc_sigma_estimate(VL53L0_DEV Dev,
				VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
				FixPoint1616_t* pSigmaEstimate)
{
	
    const uint32_t cPulseEffectiveWidth_centi_ns   = 800;
    
    const uint32_t cAmbientEffectiveWidth_centi_ns = 600;
    const FixPoint1616_t cSigmaEstRef              = 0x00000042;
    const uint32_t cVcselPulseWidth_ps             = 4700; 
    const FixPoint1616_t cSigmaEstMax              = 0x028F87AE;
    
    const FixPoint1616_t cTOF_per_mm_ps            = 0x0006999A;
    const uint32_t c16BitRoundingParam             = 0x00008000;
    const FixPoint1616_t cMaxXTalk_kcps            = 0x00320000;

    uint32_t signalTotalEventsRtn;
    uint32_t ambientEventsRtn;
    FixPoint1616_t sigmaEstimateP1;
    FixPoint1616_t sigmaEstimateP2;
    FixPoint1616_t sigmaEstimateP3;
    FixPoint1616_t deltaT_ps;
    FixPoint1616_t pwMult;
    FixPoint1616_t sigmaEstRtn;
    FixPoint1616_t sigmaEstimate;
    FixPoint1616_t xTalkCorrection;
    FixPoint1616_t ambientRate_kcps;
    FixPoint1616_t vcselRate_kcps;
    FixPoint1616_t xTalkCompRate_mcps;
    uint32_t xTalkCompRate_kcps;
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    FixPoint1616_t diff1_mcps;
    FixPoint1616_t diff2_mcps;
    FixPoint1616_t sqr1;
    FixPoint1616_t sqr2;
    FixPoint1616_t sqrSum;
    FixPoint1616_t sqrtResult_centi_ns;
    FixPoint1616_t sqrtResult;



    VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
    		xTalkCompRate_mcps);


    xTalkCompRate_kcps = xTalkCompRate_mcps * 1000;
    if(xTalkCompRate_kcps > cMaxXTalk_kcps)
    {
        xTalkCompRate_kcps = cMaxXTalk_kcps;
    }

    ambientRate_kcps = (pRangingMeasurementData->AmbientRateRtnMegaCps  * 1000) >> 16;
    vcselRate_kcps   = (pRangingMeasurementData->SignalRateRtnMegaCps * 1000) >> 16;

    Status = VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0_RdDWord(Dev, 0xc0, &signalTotalEventsRtn);
    Status |= VL53L0_RdDWord(Dev, 0xbc, &ambientEventsRtn);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);

    signalTotalEventsRtn -= 8*ambientEventsRtn;
    if (Status == VL53L0_ERROR_NONE) {
        if (vcselRate_kcps == 0) {
            *pSigmaEstimate = 0;
            PALDevDataSet(Dev, SigmaEstimate, 0);
        } else {
            if(signalTotalEventsRtn < 1)
            {
                signalTotalEventsRtn = 1;
            }


            sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

            
            sigmaEstimateP2 = (ambientRate_kcps << 16)/vcselRate_kcps;
            sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;

            sigmaEstimateP3 = 2 * VL53L0_isqrt(signalTotalEventsRtn * 12);

            
            deltaT_ps = pRangingMeasurementData->RangeMilliMeter * cTOF_per_mm_ps;

            diff1_mcps = (((vcselRate_kcps << 16) - xTalkCompRate_kcps) + 500)/1000;

            
            diff2_mcps = (((vcselRate_kcps << 16) + xTalkCompRate_kcps) + 500)/1000;

            
            diff1_mcps <<= 12;

            
            xTalkCorrection  = abs(diff1_mcps/diff2_mcps);

            
            xTalkCorrection <<= 4;

            
            pwMult = deltaT_ps/cVcselPulseWidth_ps; 

            pwMult *= ((1 << 16) - xTalkCorrection);

            
            pwMult =  (pwMult + c16BitRoundingParam) >> 16;

            
            pwMult += (1 << 16);

            pwMult >>= 1;
            
            pwMult = pwMult * pwMult;

            
            pwMult >>= 14;

            
            sqr1 = pwMult * sigmaEstimateP1;

            
            sqr1 = (sqr1 + 0x800) >> 12;

            
            sqr1 *= sqr1;

            sqr2 = sigmaEstimateP2;

            
            sqr2 = (sqr2 + 0x800) >> 12;

            
            sqr2 *= sqr2;

            
            sqrSum = sqr1 + sqr2;

            
            sqrtResult_centi_ns = VL53L0_isqrt(sqrSum);

            
            sqrtResult_centi_ns <<= 12;

            sigmaEstRtn      = (((sqrtResult_centi_ns+50)/100) / sigmaEstimateP3);
            sigmaEstRtn      *= VL53L0_SPEED_OF_LIGHT_IN_AIR;
            sigmaEstRtn      += 5000; 
            sigmaEstRtn      /= 10000;

            
            sqr1 = sigmaEstRtn * sigmaEstRtn;
            
            sqr2 = cSigmaEstRef * cSigmaEstRef;

            
            sqrtResult = VL53L0_isqrt((sqr1 + sqr2) << 8);
            sqrtResult = (sqrtResult + 0x08) >> 4;

            sigmaEstimate    = 1000 * sqrtResult;

            if((vcselRate_kcps < 1) || (signalTotalEventsRtn < 1) || (sigmaEstimate > cSigmaEstMax))
            {
                sigmaEstimate = cSigmaEstMax;
            }

            *pSigmaEstimate = (uint32_t)(sigmaEstimate);
            PALDevDataSet(Dev, SigmaEstimate, *pSigmaEstimate);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_get_pal_range_status(VL53L0_DEV Dev,
					 uint8_t DeviceRangeStatus,
					 FixPoint1616_t SignalRate,
					 uint16_t EffectiveSpadRtnCount,
					 VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
					 uint8_t* pPalRangeStatus) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t NoneFlag;
    uint8_t SigmaLimitflag = 0;
    uint8_t SignalRefClipflag = 0;
    uint8_t RangeIgnoreThresholdflag = 0;
    uint8_t SigmaLimitCheckEnable = 0;
    uint8_t SignalRateFinalRangeLimitCheckEnable = 0;
    uint8_t SignalRefClipLimitCheckEnable = 0;
    uint8_t RangeIgnoreThresholdLimitCheckEnable = 0;
    FixPoint1616_t SigmaEstimate;
    FixPoint1616_t SigmaLimitValue;
    FixPoint1616_t SignalRefClipValue;
    FixPoint1616_t RangeIgnoreThresholdValue;
    FixPoint1616_t SignalRatePerSpad;
    uint8_t DeviceRangeStatusInternal = 0;
    uint16_t tmpWord;
    uint8_t Temp8;
    FixPoint1616_t LastSignalRefMcps;
    VL53L0_DeviceParameters_t CurrentParameters;




    DeviceRangeStatusInternal = ((DeviceRangeStatus & 0x78) >> 3);

    if (DeviceRangeStatusInternal == 0 ||
		DeviceRangeStatusInternal == 5 ||
		DeviceRangeStatusInternal == 7 ||
		DeviceRangeStatusInternal == 12 ||
		DeviceRangeStatusInternal == 13 ||
		DeviceRangeStatusInternal == 14 ||
		DeviceRangeStatusInternal == 15
    		) {
    	NoneFlag = 1;
    } else {
    	NoneFlag = 0;
    }

    
    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_RdWord(Dev, VL53L0_REG_RESULT_PEAK_SIGNAL_RATE_REF,
        		&tmpWord);

    LastSignalRefMcps = VL53L0_FIXPOINT97TOFIXPOINT1616(tmpWord);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x00);

    PALDevDataSet(Dev, LastSignalRefMcps, LastSignalRefMcps);

    if (Status == VL53L0_ERROR_NONE)
    	Status =  VL53L0_GetLimitCheckEnable(Dev,
    		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, &SigmaLimitCheckEnable);

    if ((SigmaLimitCheckEnable != 0) && (Status == VL53L0_ERROR_NONE)) {
        Status = VL53L0_calc_sigma_estimate(Dev, pRangingMeasurementData,
        		&SigmaEstimate);

        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_GetLimitCheckValue(Dev,
            		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, &SigmaLimitValue);

            if ((SigmaLimitValue > 0) && (SigmaEstimate > SigmaLimitValue)) {
                
            	SigmaLimitflag = 1;
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE)
    	Status =  VL53L0_GetLimitCheckEnable(Dev,
    			VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
    			&SignalRefClipLimitCheckEnable);

    if ((SignalRefClipLimitCheckEnable != 0) && (Status == VL53L0_ERROR_NONE)) {

		Status = VL53L0_GetLimitCheckValue(Dev,
				VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, &SignalRefClipValue);

		if ((SignalRefClipValue > 0) &&
				(LastSignalRefMcps > SignalRefClipValue)) {
			
			SignalRefClipflag = 1;
		}
    }

    if (Status == VL53L0_ERROR_NONE)
    	Status =  VL53L0_GetLimitCheckEnable(Dev,
    			VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
    			&RangeIgnoreThresholdLimitCheckEnable);

    if ((RangeIgnoreThresholdLimitCheckEnable != 0) &&
    		(Status == VL53L0_ERROR_NONE)) {

		
        if (EffectiveSpadRtnCount == 0) {
        	SignalRatePerSpad = 0;
        } else {
    		SignalRatePerSpad  = (FixPoint1616_t)((256* SignalRate) /
    											EffectiveSpadRtnCount);
    	}

		Status = VL53L0_GetLimitCheckValue(Dev,
				VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
				&RangeIgnoreThresholdValue);

		if ((RangeIgnoreThresholdValue > 0) &&
				(SignalRatePerSpad < RangeIgnoreThresholdValue)) {
			
			RangeIgnoreThresholdflag = 1;
		}
    }

    if (Status == VL53L0_ERROR_NONE) {
    	if (NoneFlag == 1) {
    		*pPalRangeStatus = 255;  
    	} else if (	DeviceRangeStatusInternal == 1 ||
    				DeviceRangeStatusInternal == 2 ||
    				DeviceRangeStatusInternal == 3) {
    		*pPalRangeStatus = 5; 
    	} else if (	DeviceRangeStatusInternal == 6 ||
    				DeviceRangeStatusInternal == 9) {
    		*pPalRangeStatus = 4;  
    	} else if (	DeviceRangeStatusInternal == 8 ||
    				DeviceRangeStatusInternal == 10 ||
    				SignalRefClipflag == 1) {
    		*pPalRangeStatus = 3;  
    	} else if (	DeviceRangeStatusInternal == 4 ||
    				RangeIgnoreThresholdflag == 1) {
    		*pPalRangeStatus = 2;  
    	} else if (	SigmaLimitflag == 1) {
			*pPalRangeStatus = 1;  
		} else {
			*pPalRangeStatus = 0; 
		}
    }

    

	Status =  VL53L0_GetLimitCheckEnable(Dev,
			VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
			&SignalRateFinalRangeLimitCheckEnable);

    if (Status == VL53L0_ERROR_NONE) {
    	if ((SigmaLimitCheckEnable == 0) || (SigmaLimitflag == 1))
    		Temp8 = 1;
    	else
    		Temp8 = 0;
    	VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
    			VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, Temp8);

    	if ((DeviceRangeStatusInternal == 4) ||
    			(SignalRateFinalRangeLimitCheckEnable == 0))
    		Temp8 = 1;
    	else
    		Temp8 = 0;
    	VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
    			VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Temp8);

    	if ((SignalRefClipLimitCheckEnable == 0) || (SignalRefClipflag == 1))
    		Temp8 = 1;
    	else
    		Temp8 = 0;
    	VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
    			VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, Temp8);

    	if ((RangeIgnoreThresholdLimitCheckEnable == 0) ||
    			(RangeIgnoreThresholdflag == 1))
    		Temp8 = 1;
    	else
    		Temp8 = 0;
    	VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
    			VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD, Temp8);


    }

    return Status;

}

VL53L0_Error VL53L0_reverse_bytes(uint8_t *data, uint32_t size)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t tempData;
    uint32_t mirrorIndex;
    uint32_t middle = size/2;
    uint32_t index;

    for(index = 0; index < middle; index++)
    {
        mirrorIndex      = size - index - 1;
        tempData         = data[index];
        data[index]      = data[mirrorIndex];
        data[mirrorIndex] = tempData;
    }
    return Status;
}


VL53L0_Error VL53L0_load_tuning_settings(VL53L0_DEV Dev,
		uint8_t* pTuningSettingBuffer)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int i;
    int Index;
    uint8_t msb;
    uint8_t lsb;
    uint8_t SelectParam;
    uint8_t NumberOfWrites;
    uint8_t Address;
    uint8_t localBuffer[4]; 
    uint16_t Temp16;


    Index = 0;

    while ((*(pTuningSettingBuffer+Index) != 0) &&
    		(Status == VL53L0_ERROR_NONE)) {
        NumberOfWrites = *(pTuningSettingBuffer+Index);
        Index++;
        if (NumberOfWrites == 0xFF) {
            
            SelectParam = *(pTuningSettingBuffer+Index);
            Index++;
            switch (SelectParam) {
                case 0: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstRefArray, Temp16);
                    break;
                case 1: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstEffPulseWidth, Temp16);
                    break;
                case 2: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstEffAmbWidth, Temp16);
                    break;
                case 3: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, targetRefRate, Temp16);
                    break;
                default: 
                    Status = VL53L0_ERROR_INVALID_PARAMS;
            }

        } else if (NumberOfWrites <= 4) {
            Address = *(pTuningSettingBuffer+Index);
            Index++;

            for (i=0; i<NumberOfWrites; i++) {
                localBuffer[i] = *(pTuningSettingBuffer+Index);
                Index++;
            }

            if (NumberOfWrites == 1) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
            } else if (NumberOfWrites == 2) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
                Status = VL53L0_WrByte(Dev, Address+1, localBuffer[1]);
            } else if (NumberOfWrites == 3) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
                Status = VL53L0_WrByte(Dev, Address+1, localBuffer[1]);
                Status = VL53L0_WrByte(Dev, Address+2, localBuffer[2]);
           } else if (NumberOfWrites == 4) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
                Status = VL53L0_WrByte(Dev, Address+1, localBuffer[1]);
                Status = VL53L0_WrByte(Dev, Address+2, localBuffer[2]);
                Status = VL53L0_WrByte(Dev, Address+3, localBuffer[3]);
           }

        } else {
            Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }


    return Status;
}

VL53L0_Error VL53L0_SetReferenceSpads(VL53L0_DEV Dev,
                 uint32_t count, uint8_t isApertureSpads)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    uint32_t currentSpadIndex = 0;
    uint8_t startSelect = 0xB4;
    uint32_t spadArraySize = 6;
    uint32_t maxSpadCount = 44;
    uint32_t lastSpadIndex;
    uint32_t index;


    Status = VL53L0_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET,    0x00);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x00);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect);

    for(index = 0; index < spadArraySize; index++)
    {
        Dev->Data.SpadData.RefSpadEnables[index] = 0;
        Dev->Data.SpadData.RefGoodSpadMap[index] = 0xFF;
    }

    if(isApertureSpads)
    {
        
        while((is_aperture(startSelect + currentSpadIndex) == 0) &&
              (currentSpadIndex < maxSpadCount))
        {
            currentSpadIndex++;
        }
    }
    Status = enable_ref_spads(Dev,
                              isApertureSpads,
                              Dev->Data.SpadData.RefGoodSpadMap,
                              Dev->Data.SpadData.RefSpadEnables,
                              spadArraySize,
                              startSelect,
                              currentSpadIndex,
                              count,
                              &lastSpadIndex);

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
    	VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, count);
    	VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, isApertureSpads);
    }

    return Status;
}

VL53L0_Error VL53L0_GetReferenceSpads(VL53L0_DEV Dev,
                 uint32_t *pSpadCount, uint8_t *pIsApertureSpads)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    uint8_t refSpadsInitialised;
    uint8_t refSpadArray[6];
    uint32_t cMaxSpadCount = 44;
    uint32_t cSpadArraySize = 6;
    uint32_t spadsEnabled;
    uint8_t isApertureSpads = 0;

    refSpadsInitialised = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised);

    if(refSpadsInitialised == 1) {

        *pSpadCount       = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount);
        *pIsApertureSpads = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType);
    }
    else {

        
        Status = get_ref_spad_map(Dev, refSpadArray);

        if (Status == VL53L0_ERROR_NONE) {

            Status = count_enabled_spads(refSpadArray,
                                         cSpadArraySize,
                                         cMaxSpadCount,
                                         &spadsEnabled,
                                         &isApertureSpads);
            if (Status == VL53L0_ERROR_NONE) {

                *pSpadCount = spadsEnabled;
                *pIsApertureSpads = isApertureSpads;

                VL53L0_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
                VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, spadsEnabled);
                VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, isApertureSpads);
            }
        }
    }

    return Status;
}

VL53L0_Error VL53L0_PerformRefSpadManagement(VL53L0_DEV Dev,
                 uint32_t *refSpadCount, uint8_t *isApertureSpads)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    VL53L0_RangingMeasurementData_t rangingMeasurementData;
    uint8_t lastSpadArray[6];
    uint8_t startSelect = 0xB4;
    uint32_t minimumSpadCount = 3; 
    uint32_t maxSpadCount = 44;
    uint32_t currentSpadIndex = 0;
    uint32_t lastSpadIndex = 0;
    int32_t nextGoodSpad = 0;
    uint16_t targetRefRate = 0x0A00; 
    uint16_t peakSignalRateRef;
    uint32_t needAptSpads = 0;
    uint32_t index = 0;
    uint32_t spadArraySize = 6;
    uint32_t signalRateDiff = 0;
    uint32_t lastSignalRateDiff = 0;
    uint8_t complete;
    uint8_t SequenceConfig = 0;
    uint32_t refSpadCount_int = 0;
    uint8_t  isApertureSpads_int = 0;



    targetRefRate = PALDevDataGet(Dev, targetRefRate);

    for(index = 0; index < spadArraySize; index++)
    {
        Dev->Data.SpadData.RefSpadEnables[index] = 0;
        Dev->Data.SpadData.RefGoodSpadMap[index] = 0xFF;
    }

    Status = VL53L0_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET,    0x00);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x00);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, startSelect);


    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);

    if (Status == VL53L0_ERROR_NONE)
    {
        SequenceConfig = PALDevDataGet(Dev, SequenceConfig);

        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, 1);

        PALDevDataSet(Dev, SequenceConfig, 0x01);
    }

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_PerformSingleRangingMeasurement(Dev,
        		&rangingMeasurementData);

    if (Status == VL53L0_ERROR_NONE)
    {
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, 2);
        PALDevDataSet(Dev, SequenceConfig, 0x02);

    }
    if (Status == VL53L0_ERROR_NONE)
    {
        Status = VL53L0_PerformSingleRangingMeasurement(Dev,
        		&rangingMeasurementData);
    }

    if (Status == VL53L0_ERROR_NONE)
    {
        
        currentSpadIndex = 0;
        lastSpadIndex = currentSpadIndex;
        needAptSpads = 0;
        Status = enable_ref_spads(Dev,
                                  needAptSpads,
                                  Dev->Data.SpadData.RefGoodSpadMap,
                                  Dev->Data.SpadData.RefSpadEnables,
                                  spadArraySize,
                                  startSelect,
                                  currentSpadIndex,
                                  minimumSpadCount,
                                  &lastSpadIndex);
    }

    if (Status == VL53L0_ERROR_NONE)
    {
        currentSpadIndex = lastSpadIndex;

        Status = perform_ref_signal_measurement(Dev, &peakSignalRateRef);
            if ((Status == VL53L0_ERROR_NONE)&&(peakSignalRateRef > targetRefRate))
                {

                    for(index = 0; index < spadArraySize; index++)
                    {
                        Dev->Data.SpadData.RefSpadEnables[index] = 0;
                    }

                    
                    while((is_aperture(startSelect + currentSpadIndex) == 0) &&
                    		(currentSpadIndex < maxSpadCount))
                    {
                        currentSpadIndex++;
                    }

                    needAptSpads = 1;

                    Status = enable_ref_spads(Dev,
									  needAptSpads,
									  Dev->Data.SpadData.RefGoodSpadMap,
									  Dev->Data.SpadData.RefSpadEnables,
									  spadArraySize,
									  startSelect,
									  currentSpadIndex,
									  minimumSpadCount,
									  &lastSpadIndex);

                    if (Status == VL53L0_ERROR_NONE)
                    {
                        currentSpadIndex = lastSpadIndex;
                        Status = perform_ref_signal_measurement(Dev,
                        		&peakSignalRateRef);

                        if ((Status == VL53L0_ERROR_NONE) &&
                            (peakSignalRateRef > targetRefRate))
                        {
                            Status = VL53L0_ERROR_REF_SPAD_INIT;
                            needAptSpads    = 0;
                        }
                    }
                }
                else
                {
                    needAptSpads = 0;
                }
    }

    if (Status == VL53L0_ERROR_NONE)
    {
        if(peakSignalRateRef < targetRefRate)
        {
             isApertureSpads_int = needAptSpads;
             refSpadCount_int    = minimumSpadCount;


            memcpy(lastSpadArray, Dev->Data.SpadData.RefSpadEnables,
            		spadArraySize);
            lastSignalRateDiff = abs(peakSignalRateRef - targetRefRate);
            complete = 0;

            while(!complete)
            {
                get_next_good_spad(Dev->Data.SpadData.RefGoodSpadMap,
                		spadArraySize, currentSpadIndex, &nextGoodSpad);
                if(nextGoodSpad == -1)
                {
                    Status = VL53L0_ERROR_REF_SPAD_INIT;
                    break;
                }

                (refSpadCount_int)++;

                if(is_aperture((uint32_t)startSelect + nextGoodSpad) !=
                		needAptSpads)
                {
                    Status = VL53L0_ERROR_REF_SPAD_INIT;
                    break;
                }

                currentSpadIndex = nextGoodSpad;
                Status = enable_spad_bit(Dev->Data.SpadData.RefSpadEnables,
                		spadArraySize, currentSpadIndex);
                if (Status == VL53L0_ERROR_NONE) {
					currentSpadIndex++;
					Status = set_ref_spad_map(Dev,
							Dev->Data.SpadData.RefSpadEnables);
                }

                if (Status != VL53L0_ERROR_NONE)
                      break;

                    Status = perform_ref_signal_measurement(Dev,
                    		&peakSignalRateRef);
                if (Status != VL53L0_ERROR_NONE)
                      break;

                    signalRateDiff = abs(peakSignalRateRef -
                            		targetRefRate);
                        if(peakSignalRateRef > targetRefRate)
                        {
                            if(signalRateDiff > lastSignalRateDiff)
                            {
                                Status = set_ref_spad_map(Dev, lastSpadArray);
                                memcpy(Dev->Data.SpadData.RefSpadEnables,
                                		lastSpadArray, spadArraySize);

                                (refSpadCount_int)--;
                            }
                            complete = 1;
                        }
                        else
                        {
                            
                            lastSignalRateDiff = signalRateDiff;
                            memcpy(lastSpadArray,
                            		Dev->Data.SpadData.RefSpadEnables,
                            		spadArraySize);

                        }
            }
        }
    }

    if(Status==VL53L0_ERROR_NONE) {
                *refSpadCount = refSpadCount_int;
		*isApertureSpads = isApertureSpads_int;

            VL53L0_SETDEVICESPECIFICPARAMETER(Dev, RefSpadsInitialised, 1);
            VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadCount, (uint8_t)(*refSpadCount));
            VL53L0_SETDEVICESPECIFICPARAMETER(Dev, ReferenceSpadType, *isApertureSpads);

        
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG,
           SequenceConfig);
		if(Status==VL53L0_ERROR_NONE)
			PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
    }

    return Status;
}

VL53L0_Error perform_ref_signal_measurement(VL53L0_DEV Dev,
		uint16_t *refSignalRate)
{
    VL53L0_Error status = VL53L0_ERROR_NONE;
    VL53L0_RangingMeasurementData_t rangingMeasurementData;

    if (status == VL53L0_ERROR_NONE)
        status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, 0xC0);

    if (status == VL53L0_ERROR_NONE)
        status = VL53L0_PerformSingleRangingMeasurement(Dev,
        		&rangingMeasurementData);

    if (status == VL53L0_ERROR_NONE)
        status = VL53L0_WrByte(Dev, 0xFF, 0x01);

    if (status == VL53L0_ERROR_NONE)
        status = VL53L0_RdWord(Dev, VL53L0_REG_RESULT_PEAK_SIGNAL_RATE_REF,
        		refSignalRate);

    if (status == VL53L0_ERROR_NONE)
        status = VL53L0_WrByte(Dev, 0xFF, 0x00);

    return status;
}
VL53L0_Error enable_ref_spads(VL53L0_DEV Dev,
                              uint8_t apertureSpads,
                              uint8_t goodSpadArray[],
                              uint8_t spadArray[],
                              uint32_t size,
                              uint32_t start,
                              uint32_t offset,
                              uint32_t spadCount,
                              uint32_t *lastSpad)
{
    VL53L0_Error status = VL53L0_ERROR_NONE;
    uint32_t index;
    uint32_t i;
    int32_t nextGoodSpad = offset;
    uint32_t currentSpad;
    uint8_t checkSpadArray[6];


    currentSpad = offset;
    for(index = 0; index < spadCount; index++)
    {
        get_next_good_spad(goodSpadArray, size, currentSpad, &nextGoodSpad);
        if(nextGoodSpad == -1)
        {
            status = VL53L0_ERROR_REF_SPAD_INIT;
            break;
        }

        
        if(is_aperture(start + nextGoodSpad) != apertureSpads)
        {
            status = VL53L0_ERROR_REF_SPAD_INIT;
            break;
        }
        currentSpad = (uint32_t)nextGoodSpad;
        enable_spad_bit(spadArray, size, currentSpad);
        currentSpad++;
    }
    *lastSpad = currentSpad;

    if (status == VL53L0_ERROR_NONE)
    {
        status = set_ref_spad_map(Dev, spadArray);
    }

    if (status == VL53L0_ERROR_NONE)
    {
        status = get_ref_spad_map(Dev, checkSpadArray);

        i = 0;

        
        while(i < size)
        {
            if (spadArray[i] != checkSpadArray[i])
            {
                status = VL53L0_ERROR_REF_SPAD_INIT;
                break;
            }
            i++;
        }
    }
    return status;
}

uint8_t is_aperture(uint32_t spadIndex)
{
    uint32_t quadrant;
    uint8_t isAperture = 1;
    quadrant = spadIndex >> 6;
    if(refArrayQuadrants[quadrant] == REF_ARRAY_SPAD_0)
    {
        isAperture = 0;
    }
    return isAperture;
}

void get_next_good_spad(uint8_t goodSpadArray[], uint32_t size, uint32_t current, int32_t *next)
{
    uint32_t startIndex;
    uint32_t fineOffset;
    uint32_t cSpadsPerByte = 8;
    uint32_t coarseIndex;
    uint32_t fineIndex;
    uint8_t dataByte;
    uint8_t success = 0;


    *next = -1;

    startIndex = current / cSpadsPerByte;
    fineOffset = current % cSpadsPerByte;

    for(coarseIndex = startIndex; ((coarseIndex < size) && !success); coarseIndex++)
    {
        fineIndex = 0;
        dataByte = goodSpadArray[coarseIndex];

        if(coarseIndex == startIndex)
        {
            
            dataByte >>= fineOffset;
            fineIndex = fineOffset;
        }

        while(fineIndex < cSpadsPerByte)
        {
            if((dataByte & 0x1) == 1)
            {
                success = 1;
                *next = coarseIndex * cSpadsPerByte + fineIndex;
                break;
            }
            dataByte >>= 1;
            fineIndex++;
        }
    }
}

VL53L0_Error enable_spad_bit(uint8_t spadArray[], uint32_t size, uint32_t spadIndex)
{
    VL53L0_Error status = VL53L0_ERROR_NONE;
    uint32_t cSpadsPerByte = 8;
    uint32_t coarseIndex;
    uint32_t fineIndex;

    coarseIndex = spadIndex / cSpadsPerByte;
    fineIndex = spadIndex % cSpadsPerByte;
    if(coarseIndex >= size)
    {
        status = VL53L0_ERROR_REF_SPAD_INIT;
    }
    spadArray[coarseIndex] |= (1 << fineIndex);
    return status;
}

VL53L0_Error count_enabled_spads(uint8_t spadArray[],
        uint32_t byteCount, uint32_t maxSpads, uint32_t *pTotalSpadsEnabled, uint8_t *pIsAperture)
{
    VL53L0_Error status = VL53L0_ERROR_NONE;
    uint32_t cSpadsPerByte = 8;
    uint32_t lastByte;
    uint32_t lastBit;
    uint32_t byteIndex = 0;
    uint32_t bitIndex = 0;
    uint8_t tempByte;
    uint8_t spadTypeIdentified = 0;;


    lastByte = maxSpads / cSpadsPerByte;
    lastBit = maxSpads % cSpadsPerByte;

    
    if(lastByte >= byteCount)
    {
        status = VL53L0_ERROR_REF_SPAD_INIT;
    }

    *pTotalSpadsEnabled = 0;

    
    for(byteIndex = 0; byteIndex <= (lastByte - 1); byteIndex++)
    {
        tempByte = spadArray[byteIndex];

        for(bitIndex = 0; bitIndex <= cSpadsPerByte; bitIndex++)
        {
            if((tempByte & 0x01) == 1)
            {
                (*pTotalSpadsEnabled)++;

                if(!spadTypeIdentified)
                {
                    *pIsAperture = 1;
                    if((byteIndex < 2) && (bitIndex < 4))
                    {
                        *pIsAperture = 0;
                    }
                    spadTypeIdentified = 1;
                }
            }
            tempByte >>= 1;
        }
    }

    tempByte = spadArray[lastByte];

    for(bitIndex = 0; bitIndex <= lastBit; bitIndex++)
    {
        if((tempByte & 0x01) == 1)
        {
            (*pTotalSpadsEnabled)++;
        }
    }

    return status;
}

VL53L0_Error set_ref_spad_map(VL53L0_DEV Dev, uint8_t *refSpadArray)
{
    VL53L0_Error status = VL53L0_ERROR_NONE;
    status = VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadArray[0]);
    status = VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 1, refSpadArray[1]);
    status = VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 2, refSpadArray[2]);
    status = VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 3, refSpadArray[3]);
    status = VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 4, refSpadArray[4]);
    status = VL53L0_WrByte(Dev, VL53L0_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + 5, refSpadArray[5]);
    return status;
}

VL53L0_Error get_ref_spad_map(VL53L0_DEV Dev, uint8_t *refSpadArray)
{
    VL53L0_Error status = VL53L0_ReadMulti(Dev,
                                           VL53L0_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                                           refSpadArray,
                                           6);
    return status;
}

VL53L0_Error set_sequence_step_timeout(VL53L0_DEV Dev,
                                       VL53L0_SequenceStepId SequenceStepId,
                                       uint32_t TimeOutMicroSecs)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t CurrentVCSELPulsePeriodPClk;
    uint8_t MsrcEncodedTimeOut;
    uint16_t PreRangeEncodedTimeOut;
    uint16_t PreRangeTimeOutMClks;
    uint16_t MsrcRangeTimeOutMClks;
    uint16_t FinalRangeTimeOutMClks;
    uint16_t FinalRangeEncodedTimeOut;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;

    if((SequenceStepId == VL53L0_SEQUENCESTEP_TCC)  ||
       (SequenceStepId == VL53L0_SEQUENCESTEP_DSS)  ||
       (SequenceStepId == VL53L0_SEQUENCESTEP_MSRC)) {

        Status = VL53L0_GetVcselPulsePeriod(Dev,
                                            VL53L0_VCSEL_PERIOD_PRE_RANGE,
                                            &CurrentVCSELPulsePeriodPClk);
            if (Status == VL53L0_ERROR_NONE) {
                MsrcRangeTimeOutMClks = VL53L0_calc_timeout_mclks(Dev,
                                                                 TimeOutMicroSecs,
                                                                 (uint8_t)CurrentVCSELPulsePeriodPClk);
                if(MsrcRangeTimeOutMClks > 256) {
                    MsrcEncodedTimeOut = 255;
                }
                else {
                    MsrcEncodedTimeOut = (uint8_t)MsrcRangeTimeOutMClks - 1;
                }

                VL53L0_SETDEVICESPECIFICPARAMETER(Dev, LastEncodedTimeout, MsrcEncodedTimeOut);
            }

            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_WrByte(Dev,
                                       VL53L0_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                                       MsrcEncodedTimeOut);
            }
    }
    else {

        if(SequenceStepId == VL53L0_SEQUENCESTEP_PRE_RANGE) {

            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_GetVcselPulsePeriod(Dev,
                                                    VL53L0_VCSEL_PERIOD_PRE_RANGE,
                                                    &CurrentVCSELPulsePeriodPClk);
                PreRangeTimeOutMClks = VL53L0_calc_timeout_mclks(Dev,
                                                                 TimeOutMicroSecs,
                                                                 (uint8_t)CurrentVCSELPulsePeriodPClk);
                PreRangeEncodedTimeOut = VL53L0_encode_timeout(PreRangeTimeOutMClks);

                VL53L0_SETDEVICESPECIFICPARAMETER(Dev, LastEncodedTimeout, PreRangeEncodedTimeOut);
            }

            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_WrWord(Dev,
                                       VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                       PreRangeEncodedTimeOut);
            }
        }
        else if(SequenceStepId == VL53L0_SEQUENCESTEP_FINAL_RANGE) {


            VL53L0_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);
            PreRangeTimeOutMClks = 0;
            if(SchedulerSequenceSteps.PreRangeOn) {

                
                Status = VL53L0_GetVcselPulsePeriod(Dev,
                                                    VL53L0_VCSEL_PERIOD_PRE_RANGE,
                                                    &CurrentVCSELPulsePeriodPClk);

                
                if (Status == VL53L0_ERROR_NONE) {
                    Status = VL53L0_RdWord(Dev,
                                           VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                           &PreRangeEncodedTimeOut);
                    PreRangeTimeOutMClks = VL53L0_decode_timeout(PreRangeEncodedTimeOut);
                }
            }

            if (Status == VL53L0_ERROR_NONE) {

                Status = VL53L0_GetVcselPulsePeriod(Dev,
                                                    VL53L0_VCSEL_PERIOD_FINAL_RANGE,
                                                    &CurrentVCSELPulsePeriodPClk);
            }
            if (Status == VL53L0_ERROR_NONE) {

                FinalRangeTimeOutMClks =
                    VL53L0_calc_timeout_mclks(Dev,
                                              TimeOutMicroSecs,
                                              (uint8_t) CurrentVCSELPulsePeriodPClk);

                FinalRangeTimeOutMClks += PreRangeTimeOutMClks;

                FinalRangeEncodedTimeOut = VL53L0_encode_timeout(FinalRangeTimeOutMClks);

                if (Status == VL53L0_ERROR_NONE) {
                    Status = VL53L0_WrWord(Dev,
                                           VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                           FinalRangeEncodedTimeOut);
                }
            }
        }
        else {
            Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }
    return Status;
}

VL53L0_Error get_sequence_step_timeout(VL53L0_DEV Dev,
                                       VL53L0_SequenceStepId SequenceStepId,
                                       uint32_t *pTimeOutMicroSecs)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t CurrentVCSELPulsePeriodPClk;
    uint8_t EncodedTimeOutByte = 0;
    uint32_t TimeoutMicroSeconds = 0;
    uint16_t PreRangeEncodedTimeOut = 0;
    uint16_t MsrcTimeOutMClks;
    uint16_t PreRangeTimeOutMClks;
    uint16_t FinalRangeTimeOutMClks = 0;
    uint16_t FinalRangeEncodedTimeOut;
    VL53L0_SchedulerSequenceSteps_t SchedulerSequenceSteps;

    if (Status == VL53L0_ERROR_NONE)
    {
        if((SequenceStepId == VL53L0_SEQUENCESTEP_TCC)  ||
           (SequenceStepId == VL53L0_SEQUENCESTEP_DSS)  ||
           (SequenceStepId == VL53L0_SEQUENCESTEP_MSRC)) {

            Status = VL53L0_GetVcselPulsePeriod(Dev,
                                                VL53L0_VCSEL_PERIOD_PRE_RANGE,
                                                &CurrentVCSELPulsePeriodPClk);
            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_RdByte(Dev,
                                       VL53L0_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                                       &EncodedTimeOutByte);
            }
            MsrcTimeOutMClks = VL53L0_decode_timeout(EncodedTimeOutByte);

            TimeoutMicroSeconds = VL53L0_calc_timeout_us(Dev,
                                                         MsrcTimeOutMClks,
                                                         CurrentVCSELPulsePeriodPClk);
        }
        else if(SequenceStepId == VL53L0_SEQUENCESTEP_PRE_RANGE) {
            
            Status = VL53L0_GetVcselPulsePeriod(Dev,
                                                VL53L0_VCSEL_PERIOD_PRE_RANGE,
                                                &CurrentVCSELPulsePeriodPClk);

            
            if (Status == VL53L0_ERROR_NONE) {

                
                Status = VL53L0_GetVcselPulsePeriod(Dev,
                                VL53L0_VCSEL_PERIOD_PRE_RANGE,
                                &CurrentVCSELPulsePeriodPClk);

                if (Status == VL53L0_ERROR_NONE) {
                    Status = VL53L0_RdWord(Dev,
                                VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                &PreRangeEncodedTimeOut);
                }

                PreRangeTimeOutMClks = VL53L0_decode_timeout(PreRangeEncodedTimeOut);

                TimeoutMicroSeconds = VL53L0_calc_timeout_us(Dev,
                                PreRangeTimeOutMClks,
                                CurrentVCSELPulsePeriodPClk);
            }
        }
        else if(SequenceStepId == VL53L0_SEQUENCESTEP_FINAL_RANGE) {

            VL53L0_GetSequenceStepEnables(Dev, &SchedulerSequenceSteps);
            PreRangeTimeOutMClks = 0;

            if(SchedulerSequenceSteps.PreRangeOn) {
            
                Status = VL53L0_GetVcselPulsePeriod(Dev,
                                                    VL53L0_VCSEL_PERIOD_PRE_RANGE,
                                                    &CurrentVCSELPulsePeriodPClk);

                
                if (Status == VL53L0_ERROR_NONE) {
                    Status = VL53L0_RdWord(Dev,
                                           VL53L0_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                           &PreRangeEncodedTimeOut);
                    PreRangeTimeOutMClks = VL53L0_decode_timeout(PreRangeEncodedTimeOut);
                }
            }

            
            Status = VL53L0_GetVcselPulsePeriod(Dev,
                                                VL53L0_VCSEL_PERIOD_FINAL_RANGE,
                                                &CurrentVCSELPulsePeriodPClk);

            
            if (Status == VL53L0_ERROR_NONE) {
                Status = VL53L0_RdWord(Dev,
                                       VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                       &FinalRangeEncodedTimeOut);
                FinalRangeTimeOutMClks = VL53L0_decode_timeout(FinalRangeEncodedTimeOut);
            }

            FinalRangeTimeOutMClks -= PreRangeTimeOutMClks;
            TimeoutMicroSeconds = VL53L0_calc_timeout_us(Dev,
                                                         FinalRangeTimeOutMClks,
                                                         CurrentVCSELPulsePeriodPClk);
        }
    }

    *pTimeOutMicroSecs = TimeoutMicroSeconds;

    return Status;
}

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

#ifndef _VL53L0_API_H_
#define _VL53L0_API_H_

#include "vl53l0_strings.h"
#include "vl53l0_def.h"
#include "vl53l0_platform.h"


#ifdef __cplusplus
extern "C" {
#endif


#ifdef _MSC_VER
#   ifdef VL53L0_API_EXPORTS
#       define VL53L0_API  __declspec(dllexport)
#   else
#       define VL53L0_API
#   endif
#else
#   define VL53L0_API
#endif



VL53L0_API VL53L0_Error VL53L0_GetVersion(VL53L0_Version_t* pVersion);

VL53L0_API VL53L0_Error VL53L0_GetPalSpecVersion(
             VL53L0_Version_t* pPalSpecVersion);


VL53L0_API VL53L0_Error VL53L0_GetProductRevision(VL53L0_DEV Dev,
									uint8_t* pProductRevisionMajor,
									uint8_t* pProductRevisionMinor);

VL53L0_API VL53L0_Error VL53L0_GetDeviceInfo(VL53L0_DEV Dev,
            VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo);


VL53L0_API VL53L0_Error VL53L0_GetDeviceErrorStatus(VL53L0_DEV Dev,
            VL53L0_DeviceError* pDeviceErrorStatus);

VL53L0_API VL53L0_Error VL53L0_GetRangeStatusString(uint8_t RangeStatus,
				char* pRangeStatusString);

VL53L0_API VL53L0_Error VL53L0_GetDeviceErrorString(
            VL53L0_DeviceError ErrorCode, char* pDeviceErrorString);


VL53L0_API VL53L0_Error VL53L0_GetPalErrorString(VL53L0_Error PalErrorCode,
            char* pPalErrorString);

VL53L0_API VL53L0_Error VL53L0_GetPalStateString(VL53L0_State PalStateCode,
            char* pPalStateString);


VL53L0_API VL53L0_Error VL53L0_GetPalState(VL53L0_DEV Dev,
            VL53L0_State* pPalState);


VL53L0_API VL53L0_Error VL53L0_SetPowerMode(VL53L0_DEV Dev,
            VL53L0_PowerModes PowerMode);

VL53L0_API VL53L0_Error VL53L0_GetPowerMode(VL53L0_DEV Dev,
            VL53L0_PowerModes* pPowerMode);


VL53L0_API VL53L0_Error VL53L0_SetOffsetCalibrationDataMicroMeter(
            VL53L0_DEV Dev,
            int32_t OffsetCalibrationDataMicroMeter);

VL53L0_API VL53L0_Error VL53L0_GetOffsetCalibrationDataMicroMeter(
            VL53L0_DEV Dev,
            int32_t * pOffsetCalibrationDataMicroMeter);

VL53L0_API VL53L0_Error VL53L0_SetLinearityCorrectiveGain(
            VL53L0_DEV Dev,
            int16_t LinearityCorrectiveGain);

VL53L0_API VL53L0_Error VL53L0_GetLinearityCorrectiveGain(
            VL53L0_DEV Dev,
            int16_t * pLinearityCorrectiveGain);

VL53L0_API VL53L0_Error VL53L0_SetGroupParamHold(VL53L0_DEV Dev,
            uint8_t GroupParamHold);

VL53L0_API VL53L0_Error VL53L0_GetUpperLimitMilliMeter(VL53L0_DEV Dev,
            uint16_t* pUpperLimitMilliMeter);




VL53L0_API VL53L0_Error VL53L0_SetDeviceAddress(VL53L0_DEV Dev, uint8_t DeviceAddress);

VL53L0_API VL53L0_Error VL53L0_DataInit(VL53L0_DEV Dev);


VL53L0_Error VL53L0_SetTuningSettingBuffer(VL53L0_DEV Dev,
              uint8_t* pTuningSettingBuffer,
              uint8_t UseInternalTuningSettings);


VL53L0_Error VL53L0_GetTuningSettingBuffer(VL53L0_DEV Dev,
              uint8_t** ppTuningSettingBuffer,
              uint8_t* pUseInternalTuningSettings);



VL53L0_API VL53L0_Error VL53L0_StaticInit(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_WaitDeviceBooted(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_ResetDevice(VL53L0_DEV Dev);




VL53L0_API VL53L0_Error VL53L0_SetDeviceParameters(VL53L0_DEV Dev,
                         const VL53L0_DeviceParameters_t* pDeviceParameters);

VL53L0_API VL53L0_Error VL53L0_GetDeviceParameters(VL53L0_DEV Dev,
            VL53L0_DeviceParameters_t* pDeviceParameters);

VL53L0_API VL53L0_Error VL53L0_SetDeviceMode(VL53L0_DEV Dev,
            VL53L0_DeviceModes DeviceMode);

VL53L0_API VL53L0_Error VL53L0_GetDeviceMode(VL53L0_DEV Dev,
            VL53L0_DeviceModes* pDeviceMode);

VL53L0_API VL53L0_Error VL53L0_SetHistogramMode(VL53L0_DEV Dev,
                         VL53L0_HistogramModes HistogramMode);

VL53L0_API VL53L0_Error VL53L0_GetHistogramMode(VL53L0_DEV Dev,
                         VL53L0_HistogramModes* pHistogramMode);

VL53L0_API VL53L0_Error VL53L0_SetMeasurementTimingBudgetMicroSeconds(VL53L0_DEV Dev,
                         uint32_t  MeasurementTimingBudgetMicroSeconds);

VL53L0_API VL53L0_Error VL53L0_GetMeasurementTimingBudgetMicroSeconds(VL53L0_DEV Dev,
                     uint32_t*  pMeasurementTimingBudgetMicroSeconds);

VL53L0_API VL53L0_Error VL53L0_GetVcselPulsePeriod(VL53L0_DEV Dev,
                    VL53L0_VcselPeriod VcselPeriodType, uint8_t* pVCSELPulsePeriod);

VL53L0_API VL53L0_Error VL53L0_SetVcselPulsePeriod(VL53L0_DEV Dev,
                    VL53L0_VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriod);

VL53L0_API VL53L0_Error VL53L0_SetSequenceStepEnable(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled);

VL53L0_API VL53L0_Error VL53L0_GetSequenceStepEnable(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, uint8_t* pSequenceStepEnabled);

VL53L0_API VL53L0_Error VL53L0_GetSequenceStepEnables(VL53L0_DEV Dev,
                    VL53L0_SchedulerSequenceSteps_t *pSchedulerSequenceSteps);

VL53L0_API VL53L0_Error VL53L0_SetSequenceStepTimeout(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, FixPoint1616_t TimeOutMilliSecs);

VL53L0_API VL53L0_Error VL53L0_GetSequenceStepTimeout(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId, FixPoint1616_t *pTimeOutMilliSecs);

VL53L0_API VL53L0_Error VL53L0_GetNumberOfSequenceSteps(VL53L0_DEV Dev,
                    uint8_t* pNumberOfSequenceSteps);

VL53L0_API VL53L0_Error VL53L0_GetSequenceStepsInfo(
                    VL53L0_SequenceStepId SequenceStepId,
                    char* pSequenceStepsString);

VL53L0_API VL53L0_Error VL53L0_SetInterMeasurementPeriodMilliSeconds(VL53L0_DEV Dev,
                         uint32_t InterMeasurementPeriodMilliSeconds);

VL53L0_API VL53L0_Error VL53L0_GetInterMeasurementPeriodMilliSeconds(
            VL53L0_DEV Dev, uint32_t* pInterMeasurementPeriodMilliSeconds);

VL53L0_API VL53L0_Error VL53L0_SetXTalkCompensationEnable(VL53L0_DEV Dev,
                         uint8_t XTalkCompensationEnable);

VL53L0_API VL53L0_Error VL53L0_GetXTalkCompensationEnable(VL53L0_DEV Dev,
                         uint8_t* pXTalkCompensationEnable);

VL53L0_API VL53L0_Error VL53L0_SetXTalkCompensationRateMegaCps(VL53L0_DEV Dev,
                         FixPoint1616_t XTalkCompensationRateMegaCps);

VL53L0_API VL53L0_Error VL53L0_GetXTalkCompensationRateMegaCps(VL53L0_DEV Dev,
                         FixPoint1616_t* pXTalkCompensationRateMegaCps);


VL53L0_API VL53L0_Error VL53L0_SetRefCalibration(VL53L0_DEV Dev,
		uint8_t VhvSettings, uint8_t PhaseCal);

VL53L0_API VL53L0_Error VL53L0_GetRefCalibration(VL53L0_DEV Dev,
		uint8_t *pVhvSettings, uint8_t *pPhaseCal);

VL53L0_API VL53L0_Error VL53L0_GetNumberOfLimitCheck(
            uint16_t* pNumberOfLimitCheck);

VL53L0_API VL53L0_Error VL53L0_GetLimitCheckInfo(VL53L0_DEV Dev,
            uint16_t LimitCheckId, char* pLimitCheckString);


VL53L0_API VL53L0_Error VL53L0_GetLimitCheckStatus(VL53L0_DEV Dev,
            uint16_t LimitCheckId, uint8_t* pLimitCheckStatus);


VL53L0_API VL53L0_Error VL53L0_SetLimitCheckEnable(VL53L0_DEV Dev,
            uint16_t LimitCheckId, uint8_t LimitCheckEnable);


VL53L0_API VL53L0_Error VL53L0_GetLimitCheckEnable(VL53L0_DEV Dev,
            uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);

VL53L0_API VL53L0_Error VL53L0_SetLimitCheckValue(VL53L0_DEV Dev,
            uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue);

VL53L0_API VL53L0_Error VL53L0_GetLimitCheckValue(VL53L0_DEV Dev,
            uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue);

VL53L0_API VL53L0_Error VL53L0_GetLimitCheckCurrent(VL53L0_DEV Dev,
		uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);


VL53L0_API VL53L0_Error VL53L0_SetWrapAroundCheckEnable(VL53L0_DEV Dev,
            uint8_t WrapAroundCheckEnable);

VL53L0_API VL53L0_Error VL53L0_GetWrapAroundCheckEnable(VL53L0_DEV Dev,
            uint8_t* pWrapAroundCheckEnable);




VL53L0_API VL53L0_Error VL53L0_PerformSingleMeasurement(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_PerformRefCalibration(VL53L0_DEV Dev,
		  uint8_t *pVhvSettings, uint8_t *pPhaseCal);

VL53L0_API VL53L0_Error VL53L0_PerformXTalkCalibration(VL53L0_DEV Dev,
            FixPoint1616_t XTalkCalDistance,
            FixPoint1616_t* pXTalkCompensationRateMegaCps);

VL53L0_API VL53L0_Error VL53L0_PerformOffsetCalibration(VL53L0_DEV Dev,
            FixPoint1616_t CalDistanceMilliMeter, int32_t* pOffsetMicroMeter);

VL53L0_API VL53L0_Error VL53L0_StartMeasurement(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_StopMeasurement(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_GetMeasurementDataReady(VL53L0_DEV Dev,
            uint8_t *pMeasurementDataReady);

VL53L0_API VL53L0_Error VL53L0_WaitDeviceReadyForNewMeasurement(VL53L0_DEV Dev,
            uint32_t MaxLoop);


VL53L0_API VL53L0_Error VL53L0_GetMeasurementRefSignal(VL53L0_DEV Dev,
		FixPoint1616_t *pMeasurementRefSignal);


VL53L0_API VL53L0_Error VL53L0_GetRangingMeasurementData(VL53L0_DEV Dev,
            VL53L0_RangingMeasurementData_t *pRangingMeasurementData);

VL53L0_API VL53L0_Error VL53L0_GetHistogramMeasurementData(VL53L0_DEV Dev,
            VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData);


VL53L0_API VL53L0_Error VL53L0_PerformSingleRangingMeasurement(VL53L0_DEV Dev,
            VL53L0_RangingMeasurementData_t *pRangingMeasurementData);


VL53L0_API VL53L0_Error VL53L0_PerformSingleHistogramMeasurement(VL53L0_DEV Dev,
            VL53L0_HistogramMeasurementData_t* pHistogramMeasurementData);



VL53L0_API VL53L0_Error VL53L0_SetNumberOfROIZones(VL53L0_DEV Dev,
            uint8_t NumberOfROIZones);

VL53L0_API VL53L0_Error VL53L0_GetNumberOfROIZones(VL53L0_DEV Dev,
            uint8_t* pNumberOfROIZones);

VL53L0_API VL53L0_Error VL53L0_GetMaxNumberOfROIZones(VL53L0_DEV Dev,
            uint8_t* pMaxNumberOfROIZones);





VL53L0_API VL53L0_Error VL53L0_SetGpioConfig(VL53L0_DEV Dev,
            uint8_t Pin,
            VL53L0_DeviceModes DeviceMode,
            VL53L0_GpioFunctionality Functionality,
            VL53L0_InterruptPolarity Polarity);


VL53L0_API VL53L0_Error VL53L0_GetGpioConfig(VL53L0_DEV Dev,
            uint8_t Pin,
            VL53L0_DeviceModes* pDeviceMode,
            VL53L0_GpioFunctionality* pFunctionality,
            VL53L0_InterruptPolarity* pPolarity);


VL53L0_API VL53L0_Error VL53L0_SetInterruptThresholds(VL53L0_DEV Dev,
            VL53L0_DeviceModes DeviceMode,
            FixPoint1616_t ThresholdLow,
            FixPoint1616_t ThresholdHigh);

VL53L0_API VL53L0_Error VL53L0_GetInterruptThresholds(VL53L0_DEV Dev,
            VL53L0_DeviceModes DeviceMode,
            FixPoint1616_t* pThresholdLow,
            FixPoint1616_t* pThresholdHigh);

VL53L0_API VL53L0_Error VL53L0_ClearInterruptMask(VL53L0_DEV Dev, uint32_t InterruptMask);

VL53L0_API VL53L0_Error VL53L0_GetInterruptMaskStatus(VL53L0_DEV Dev,
            uint32_t *pInterruptMaskStatus);


VL53L0_API VL53L0_Error VL53L0_EnableInterruptMask(VL53L0_DEV Dev, uint32_t InterruptMask);







VL53L0_API VL53L0_Error VL53L0_SetSpadAmbientDamperThreshold(VL53L0_DEV Dev,
            uint16_t SpadAmbientDamperThreshold);

VL53L0_API VL53L0_Error VL53L0_GetSpadAmbientDamperThreshold(VL53L0_DEV Dev,
            uint16_t* pSpadAmbientDamperThreshold);


VL53L0_API VL53L0_Error VL53L0_SetSpadAmbientDamperFactor(VL53L0_DEV Dev,
            uint16_t SpadAmbientDamperFactor);

VL53L0_API VL53L0_Error VL53L0_GetSpadAmbientDamperFactor(VL53L0_DEV Dev,
                    uint16_t* pSpadAmbientDamperFactor);


VL53L0_API VL53L0_Error VL53L0_PerformRefSpadManagement(VL53L0_DEV Dev,
                     uint32_t *refSpadCount, uint8_t *isApertureSpads);

 VL53L0_API VL53L0_Error VL53L0_SetReferenceSpads(VL53L0_DEV Dev,
                      uint32_t refSpadCount, uint8_t isApertureSpads);

 VL53L0_API VL53L0_Error VL53L0_GetReferenceSpads(VL53L0_DEV Dev,
                      uint32_t *refSpadCount, uint8_t *isApertureSpads);




#ifdef __cplusplus
}
#endif

#endif 

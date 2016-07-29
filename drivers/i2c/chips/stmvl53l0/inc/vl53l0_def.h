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



#ifndef _VL53L0_DEF_H_
#define _VL53L0_DEF_H_


#ifdef __cplusplus
extern "C" {
#endif


#define VL53L010_SPECIFICATION_VER_MAJOR   1
#define VL53L010_SPECIFICATION_VER_MINOR   2
#define VL53L010_SPECIFICATION_VER_SUB     7
#define VL53L010_SPECIFICATION_VER_REVISION 1440

#define VL53L010_IMPLEMENTATION_VER_MAJOR   1
#define VL53L010_IMPLEMENTATION_VER_MINOR   0
#define VL53L010_IMPLEMENTATION_VER_SUB     9
#define VL53L010_IMPLEMENTATION_VER_REVISION    3673

#define VL53L0_SPECIFICATION_VER_MAJOR   1
#define VL53L0_SPECIFICATION_VER_MINOR   2
#define VL53L0_SPECIFICATION_VER_SUB     7
#define VL53L0_SPECIFICATION_VER_REVISION 1440

#define VL53L0_IMPLEMENTATION_VER_MAJOR   1
#define VL53L0_IMPLEMENTATION_VER_MINOR   1
#define VL53L0_IMPLEMENTATION_VER_SUB     13
#define VL53L0_IMPLEMENTATION_VER_REVISION    3771
#define VL53L0_DEFAULT_MAX_LOOP 100
#define VL53L0_MAX_STRING_LENGTH 32


#include "vl53l0_device.h"
#include "vl53l0_types.h"



typedef struct{
	uint32_t     revision; 
	uint8_t      major;    
	uint8_t      minor;    
	uint8_t      build;    
} VL53L0_Version_t;


typedef struct {
    char Name[VL53L0_MAX_STRING_LENGTH]; 
    char Type[VL53L0_MAX_STRING_LENGTH]; 
    char ProductId[VL53L0_MAX_STRING_LENGTH]; 
    uint8_t ProductType; 
    uint8_t ProductRevisionMajor; 
    uint8_t ProductRevisionMinor; 
} VL53L0_DeviceInfo_t;



typedef int8_t VL53L0_Error;

#define VL53L0_ERROR_NONE                              ((VL53L0_Error)  0)
#define VL53L0_ERROR_CALIBRATION_WARNING               ((VL53L0_Error) -1)    
#define VL53L0_ERROR_MIN_CLIPPED                       ((VL53L0_Error) -2)    

#define VL53L0_ERROR_UNDEFINED                         ((VL53L0_Error) -3)    
#define VL53L0_ERROR_INVALID_PARAMS                    ((VL53L0_Error) -4)    
#define VL53L0_ERROR_NOT_SUPPORTED                     ((VL53L0_Error) -5)    
#define VL53L0_ERROR_RANGE_ERROR                       ((VL53L0_Error) -6)    
#define VL53L0_ERROR_TIME_OUT                          ((VL53L0_Error) -7)    
#define VL53L0_ERROR_MODE_NOT_SUPPORTED                ((VL53L0_Error) -8)    
#define VL53L0_ERROR_BUFFER_TOO_SMALL                  ((VL53L0_Error) -9)    
#define VL53L0_ERROR_GPIO_NOT_EXISTING                 ((VL53L0_Error) -10)   
#define VL53L0_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED  ((VL53L0_Error) -11)   
#define VL53L0_ERROR_CONTROL_INTERFACE                 ((VL53L0_Error) -20)   
#define VL53L0_ERROR_INVALID_COMMAND                   ((VL53L0_Error) -30)   
#define VL53L0_ERROR_DIVISION_BY_ZERO                  ((VL53L0_Error) -40)   
#define VL53L0_ERROR_REF_SPAD_INIT                     ((VL53L0_Error) -50)   
#define VL53L0_ERROR_NOT_IMPLEMENTED                   ((VL53L0_Error) -99)   
 


typedef uint8_t VL53L0_DeviceModes;

#define VL53L0_DEVICEMODE_SINGLE_RANGING           ((VL53L0_DeviceModes)  0)
#define VL53L0_DEVICEMODE_CONTINUOUS_RANGING       ((VL53L0_DeviceModes)  1)
#define VL53L0_DEVICEMODE_SINGLE_HISTOGRAM         ((VL53L0_DeviceModes)  2)
#define VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING ((VL53L0_DeviceModes)  3)
#define VL53L0_DEVICEMODE_SINGLE_ALS               ((VL53L0_DeviceModes) 10)
#define VL53L0_DEVICEMODE_GPIO_DRIVE               ((VL53L0_DeviceModes) 20)
#define VL53L0_DEVICEMODE_GPIO_OSC                 ((VL53L0_DeviceModes) 21)
	
 



typedef uint8_t VL53L0_HistogramModes;

#define VL53L0_HISTOGRAMMODE_DISABLED        ((VL53L0_HistogramModes) 0) 
#define VL53L0_HISTOGRAMMODE_REFERENCE_ONLY  ((VL53L0_HistogramModes) 1) 
#define VL53L0_HISTOGRAMMODE_RETURN_ONLY     ((VL53L0_HistogramModes) 2) 
#define VL53L0_HISTOGRAMMODE_BOTH            ((VL53L0_HistogramModes) 3) 
	
 



typedef uint8_t VL53L0_PowerModes;

#define VL53L0_POWERMODE_STANDBY_LEVEL1 ((VL53L0_PowerModes) 0)   
#define VL53L0_POWERMODE_STANDBY_LEVEL2 ((VL53L0_PowerModes) 1)   
#define VL53L0_POWERMODE_IDLE_LEVEL1    ((VL53L0_PowerModes) 2)   
#define VL53L0_POWERMODE_IDLE_LEVEL2    ((VL53L0_PowerModes) 3)   

 


typedef struct{
	VL53L0_DeviceModes DeviceMode;
    
    VL53L0_HistogramModes HistogramMode;
    
	uint32_t MeasurementTimingBudgetMicroSeconds;
    
	uint32_t InterMeasurementPeriodMilliSeconds;
	uint8_t XTalkCompensationEnable;
    
	uint16_t XTalkCompensationRangeMilliMeter;
    
	FixPoint1616_t XTalkCompensationRateMegaCps;
    
	int32_t RangeOffsetMicroMeters;
    

	uint8_t LimitChecksEnable[VL53L0_CHECKENABLE_NUMBER_OF_CHECKS];
    
	uint8_t LimitChecksStatus[VL53L0_CHECKENABLE_NUMBER_OF_CHECKS];
	FixPoint1616_t LimitChecksValue[VL53L0_CHECKENABLE_NUMBER_OF_CHECKS];
    

	uint8_t WrapAroundCheckEnable;
    
} VL53L0_DeviceParameters_t;



typedef uint8_t VL53L0_State;

#define VL53L0_STATE_POWERDOWN       ((VL53L0_State)  0)  
#define VL53L0_STATE_WAIT_STATICINIT ((VL53L0_State)  1)  
#define VL53L0_STATE_STANDBY         ((VL53L0_State)  2)  
#define VL53L0_STATE_IDLE            ((VL53L0_State)  3)  
#define VL53L0_STATE_RUNNING         ((VL53L0_State)  4)  
#define VL53L0_STATE_UNKNOWN         ((VL53L0_State)  98) 
#define VL53L0_STATE_ERROR           ((VL53L0_State)  99) 

 


typedef struct{
	int32_t AmbTuningWindowFactor_K; 
    int32_t RetSignalAt0mm;          
} VL53L0_DMaxData_t;

typedef struct{
    uint32_t TimeStamp;                   
	uint32_t MeasurementTimeUsec;         


	uint16_t RangeMilliMeter;             

	uint16_t RangeDMaxMilliMeter;         

	FixPoint1616_t SignalRateRtnMegaCps;  
	FixPoint1616_t AmbientRateRtnMegaCps; 

	uint16_t EffectiveSpadRtnCount;       

	uint8_t ZoneId;                       
	uint8_t RangeFractionalPart;          
	uint8_t RangeStatus;                  
} VL53L0_RangingMeasurementData_t;


#define VL53L0_HISTOGRAM_BUFFER_SIZE 24

typedef struct{
	
	uint32_t HistogramData[VL53L0_HISTOGRAM_BUFFER_SIZE];
	
	uint8_t HistogramType; 
	uint8_t FirstBin; 
	uint8_t BufferSize; 
	uint8_t NumberOfBins;
	

	VL53L0_DeviceError ErrorStatus;
} VL53L0_HistogramMeasurementData_t;

#define VL53L0_REF_SPAD_BUFFER_SIZE 6

typedef struct{
	uint8_t RefSpadEnables[VL53L0_REF_SPAD_BUFFER_SIZE];
	
	uint8_t RefGoodSpadMap[VL53L0_REF_SPAD_BUFFER_SIZE];
	
} VL53L0_SpadData_t;

typedef struct{
    FixPoint1616_t OscFrequencyMHz; 

    uint16_t LastEncodedTimeout; 

    VL53L0_GpioFunctionality Pin0GpioFunctionality; 

    uint8_t ReadDataFromDeviceDone; 
    uint8_t ModuleId; 
    uint8_t Revision; 
    char ProductId[VL53L0_MAX_STRING_LENGTH]; 
    uint8_t ReferenceSpadCount; 
    uint8_t ReferenceSpadType;  
    uint8_t RefSpadsInitialised; 
    uint32_t PartUIDUpper; 
    uint32_t PartUIDLower; 

} VL53L0_DeviceSpecificParameters_t;

typedef struct {
	VL53L0_DMaxData_t DMaxData;
	
	int32_t  Part2PartOffsetNVMMicroMeter;
	
	int32_t  Part2PartOffsetAdjustmentNVMMicroMeter;
	
	VL53L0_DeviceParameters_t CurrentParameters;
	
	VL53L0_RangingMeasurementData_t LastRangeMeasure;
	
	VL53L0_HistogramMeasurementData_t LastHistogramMeasure;
	
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    
    VL53L0_SpadData_t SpadData;
    
    uint8_t SequenceConfig;
    
    uint8_t RangeFractionalEnable;
    
    VL53L0_State PalState;
    
    VL53L0_PowerModes PowerMode;
    
    uint16_t SigmaEstRefArray;
    
    uint16_t SigmaEstEffPulseWidth;
    uint16_t SigmaEstEffAmbWidth;
    uint16_t targetRefRate;
    
    FixPoint1616_t SigmaEstimate;
	FixPoint1616_t SignalEstimate;
	
    FixPoint1616_t LastSignalRefMcps;
    
    uint8_t *pTuningSettingsPointer;
    
    uint8_t UseInternalTuningSettings;
    
    uint16_t LinearityCorrectiveGain;
    


} VL53L0_DevData_t;



typedef uint8_t VL53L0_InterruptPolarity;

#define VL53L0_INTERRUPTPOLARITY_LOW       ((VL53L0_InterruptPolarity)  0)
#define VL53L0_INTERRUPTPOLARITY_HIGH      ((VL53L0_InterruptPolarity)  1)

 


typedef uint8_t VL53L0_VcselPeriod;

#define VL53L0_VCSEL_PERIOD_PRE_RANGE   ((VL53L0_VcselPeriod) 0)
#define VL53L0_VCSEL_PERIOD_FINAL_RANGE ((VL53L0_VcselPeriod) 1)

 

typedef struct{
    uint8_t      TccOn;        
    uint8_t      MsrcOn;       
    uint8_t      DssOn;        
    uint8_t      PreRangeOn;   
    uint8_t      FinalRangeOn; 
} VL53L0_SchedulerSequenceSteps_t;

 

typedef uint8_t VL53L0_SequenceStepId;

#define  VL53L0_SEQUENCESTEP_TCC         ((VL53L0_VcselPeriod) 0)
#define  VL53L0_SEQUENCESTEP_DSS         ((VL53L0_VcselPeriod) 1)
#define  VL53L0_SEQUENCESTEP_MSRC        ((VL53L0_VcselPeriod) 2)
#define  VL53L0_SEQUENCESTEP_PRE_RANGE   ((VL53L0_VcselPeriod) 3)
#define  VL53L0_SEQUENCESTEP_FINAL_RANGE ((VL53L0_VcselPeriod) 4)

#define  VL53L0_SEQUENCESTEP_NUMBER_OF_CHECKS            5

 

#ifdef __cplusplus
}
#endif


#endif 

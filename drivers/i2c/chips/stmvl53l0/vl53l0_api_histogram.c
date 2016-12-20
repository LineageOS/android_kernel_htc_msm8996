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
#include "inc/vl53l0_api_histogram.h"


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

typedef uint32_t WindowSelection;
#define VL53L0_AMBIENT_WINDOW_ONLY		 ((WindowSelection) 0)
	
#define VL53L0_AMBIENT_AND_SIGNAL_WINDOW ((WindowSelection) 1)
	


VL53L0_Error VL53L0_start_histogram_measurement(VL53L0_DEV Dev,
			VL53L0_HistogramModes histoMode,
			uint32_t count)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	uint8_t dataByte;
	


	dataByte = VL53L0_REG_SYSRANGE_MODE_SINGLESHOT |
		VL53L0_REG_SYSRANGE_MODE_START_STOP;

	
	if (count == 0)
		dataByte |= (1 << 5);

	switch (histoMode) {
	case VL53L0_HISTOGRAMMODE_DISABLED:
		
		Status = VL53L0_ERROR_INVALID_COMMAND;
		break;

	case VL53L0_HISTOGRAMMODE_REFERENCE_ONLY:
	case VL53L0_HISTOGRAMMODE_RETURN_ONLY:
	case VL53L0_HISTOGRAMMODE_BOTH:
		dataByte |= (histoMode << 3);
		Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START,
			dataByte);
		if (Status == VL53L0_ERROR_NONE) {
			
			PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);
		}
		break;

	default:
		
		Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
	}

	
	return Status;
}

VL53L0_Error VL53L0_confirm_measurement_start(VL53L0_DEV Dev)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	uint8_t NewDataReady = 0;
	uint32_t LoopNb;

	

	LoopNb = 0;
	do {
		Status = VL53L0_GetMeasurementDataReady(Dev, &NewDataReady);
		if ((NewDataReady == 0x01) || Status != VL53L0_ERROR_NONE)
				break;

		LoopNb = LoopNb + 1;
		VL53L0_PollingDelay(Dev);
	} while (LoopNb < VL53L0_DEFAULT_MAX_LOOP);

	if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP)
		Status = VL53L0_ERROR_TIME_OUT;

	

	return Status;
}


VL53L0_Error VL53L0_set_histogram_mode(VL53L0_DEV Dev,
		VL53L0_HistogramModes HistogramMode)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	switch (HistogramMode) {
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

VL53L0_Error VL53L0_get_histogram_mode(VL53L0_DEV Dev,
	VL53L0_HistogramModes *pHistogramMode)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	VL53L0_GETPARAMETERFIELD(Dev, HistogramMode, *pHistogramMode);

	return Status;
}


VL53L0_Error VL53L0_perform_single_histogram_measurement(VL53L0_DEV Dev,
		VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	VL53L0_DeviceModes DeviceMode;
	VL53L0_HistogramModes HistogramMode = VL53L0_HISTOGRAMMODE_DISABLED;
	uint32_t MeasCount;
	uint32_t Measurements;

	
	Status = VL53L0_GetHistogramMode(Dev, &HistogramMode);


	if (Status != VL53L0_ERROR_NONE)
		return Status;


	if (HistogramMode == VL53L0_HISTOGRAMMODE_BOTH) {
		if (pHistogramMeasurementData->BufferSize <
				VL53L0_HISTOGRAM_BUFFER_SIZE) {
			Status = VL53L0_ERROR_BUFFER_TOO_SMALL;
		}
	} else {
		if (pHistogramMeasurementData->BufferSize <
				VL53L0_HISTOGRAM_BUFFER_SIZE/2) {
			Status = VL53L0_ERROR_BUFFER_TOO_SMALL;
		}
	}
	pHistogramMeasurementData->HistogramType = (uint8_t)HistogramMode;
	pHistogramMeasurementData->ErrorStatus	 = VL53L0_DEVICEERROR_NONE;
	pHistogramMeasurementData->FirstBin		 = 0;
	pHistogramMeasurementData->NumberOfBins	 = 0;


	
	if (Status == VL53L0_ERROR_NONE)
		Status = VL53L0_GetDeviceMode(Dev, &DeviceMode);

	if (Status == VL53L0_ERROR_NONE)
		Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_HISTOGRAM_BIN,
			0x00);

	if (Status == VL53L0_ERROR_NONE)
		Status = VL53L0_WrByte(Dev,
			VL53L0_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT, 0x00);

	if (Status == VL53L0_ERROR_NONE)
		Status = VL53L0_WrByte(Dev,
				VL53L0_REG_HISTOGRAM_CONFIG_READOUT_CTRL, 0x01);

	if (Status != VL53L0_ERROR_NONE)
		return Status;

	Measurements = 3;
	if (HistogramMode == VL53L0_HISTOGRAMMODE_BOTH)
		Measurements = 6;

	if (DeviceMode != VL53L0_DEVICEMODE_SINGLE_HISTOGRAM) {
		Status = VL53L0_ERROR_INVALID_COMMAND;
		return Status;
	}

	
	MeasCount = 0;
	while ((MeasCount < Measurements) && (Status == VL53L0_ERROR_NONE)) {
		Status = VL53L0_start_histogram_measurement(Dev, HistogramMode,
			MeasCount);

		if (Status == VL53L0_ERROR_NONE)
			VL53L0_confirm_measurement_start(Dev);

		if (Status == VL53L0_ERROR_NONE)
			PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);

		if (Status == VL53L0_ERROR_NONE)
			Status = VL53L0_measurement_poll_for_completion(Dev);

		if (Status == VL53L0_ERROR_NONE) {
			Status = VL53L0_read_histo_measurement(Dev,
			pHistogramMeasurementData->HistogramData,
			MeasCount,
			HistogramMode);

			if (Status == VL53L0_ERROR_NONE) {
				if (HistogramMode == VL53L0_HISTOGRAMMODE_BOTH)
					pHistogramMeasurementData->NumberOfBins
						+= 2;
				else
					pHistogramMeasurementData->NumberOfBins
						+= 4;

			}
		}

		if (Status == VL53L0_ERROR_NONE)
			Status = VL53L0_ClearInterruptMask(Dev, 0);

		MeasCount++;
	}

	
	if (Status == VL53L0_ERROR_NONE) {
		pHistogramMeasurementData->NumberOfBins = 12;
		PALDevDataSet(Dev, PalState, VL53L0_STATE_IDLE);
	}

	return Status;
}


VL53L0_Error VL53L0_get_histogram_measurement_data(VL53L0_DEV Dev,
		VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData)
{
	VL53L0_Error Status = VL53L0_ERROR_NOT_IMPLEMENTED;
	

	
	return Status;
}

VL53L0_Error VL53L0_read_histo_measurement(VL53L0_DEV Dev,
			uint32_t *histoData,
			uint32_t offset,
			VL53L0_HistogramModes histoMode)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	uint8_t localBuffer[28];
	uint32_t cDataSize	= 4;
	uint32_t offset1;

	

	Status = VL53L0_WrByte(Dev, 0xFF, VL53L0_REG_RESULT_CORE_PAGE);
	Status = VL53L0_ReadMulti(Dev,
		(uint8_t)VL53L0_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN,
		localBuffer,
		28);
	Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);

	if (Status == VL53L0_ERROR_NONE) {
		VL53L0_reverse_bytes(&localBuffer[0], cDataSize);
		VL53L0_reverse_bytes(&localBuffer[4], cDataSize);
		VL53L0_reverse_bytes(&localBuffer[20], cDataSize);
		VL53L0_reverse_bytes(&localBuffer[24], cDataSize);

		offset1 = offset * cDataSize;
		if (histoMode == VL53L0_HISTOGRAMMODE_BOTH) {

			memcpy(&histoData[offset1], &localBuffer[4],
				cDataSize); 
			memcpy(&histoData[offset1 + 1], &localBuffer[24],
				cDataSize); 
			memcpy(&histoData[offset1 + 2], &localBuffer[0],
				cDataSize); 
			memcpy(&histoData[offset1 + 3], &localBuffer[20],
				cDataSize); 

		} else {

			memcpy(&histoData[offset1], &localBuffer[24],
				cDataSize);
			memcpy(&histoData[offset1 + 1], &localBuffer[20],
				cDataSize);
			memcpy(&histoData[offset1 + 2], &localBuffer[4],
				cDataSize);
			memcpy(&histoData[offset1 + 3], &localBuffer[0],
				cDataSize);

		}
	}

	

	return Status;
}


VL53L0_Error VL53L0_get_max_spads(VL53L0_DEV Dev,
	uint32_t *pmax_spads, uint8_t *pambient_too_high)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	uint8_t TCC_Enabled;
	uint8_t MSRC_Enabled;
	VL53L0_RangingMeasurementData_t RangingMeasurementData;
	FixPoint1616_t ratio = 0;
	uint32_t max_spads = 0;

	
	if (Status == VL53L0_ERROR_NONE)
		Status = VL53L0_GetSequenceStepEnable(Dev,
				VL53L0_SEQUENCESTEP_TCC, &TCC_Enabled);

	
	if (Status == VL53L0_ERROR_NONE)
		Status = VL53L0_GetSequenceStepEnable(Dev,
				VL53L0_SEQUENCESTEP_MSRC, &MSRC_Enabled);

	
	if  ((Status == VL53L0_ERROR_NONE) && (TCC_Enabled != 0))
		Status = VL53L0_SetSequenceStepEnable(Dev,
				VL53L0_SEQUENCESTEP_TCC, 0);

	
	if ((Status == VL53L0_ERROR_NONE) && (MSRC_Enabled != 0))
		Status = VL53L0_SetSequenceStepEnable(Dev,
				VL53L0_SEQUENCESTEP_MSRC, 0);


	if (Status == VL53L0_ERROR_NONE) {
		Status = VL53L0_PerformSingleRangingMeasurement(Dev,
				&RangingMeasurementData);
		max_spads = (RangingMeasurementData.EffectiveSpadRtnCount +
			128)/256;
		*pmax_spads = max_spads;
	}

	
	if (Status == VL53L0_ERROR_NONE) {
		if (max_spads <= 0) {
			*pambient_too_high = 1;
			return VL53L0_ERROR_DIVISION_BY_ZERO;
		} else {
			ratio = RangingMeasurementData.AmbientRateRtnMegaCps /
				max_spads;

			if (ratio >  65536/100)
				*pambient_too_high = 1;
			else
				*pambient_too_high = 0;

		}
	}


	
	if (Status == VL53L0_ERROR_NONE) {
		if (TCC_Enabled != 0)
			Status = VL53L0_SetSequenceStepEnable(Dev,
					VL53L0_SEQUENCESTEP_TCC, 1);
	}

	
	if (Status == VL53L0_ERROR_NONE) {
		if (MSRC_Enabled != 0)
			Status = VL53L0_SetSequenceStepEnable(Dev,
					VL53L0_SEQUENCESTEP_MSRC, 1);
	}

	return Status;

}


VL53L0_Error calc_xtalk_mcps_per_spad(
	uint32_t rtn_signal_events,
	uint32_t timeout_ms,
	uint32_t max_spads,
	uint8_t vcsel_pulse_period_pclk,
	FixPoint1616_t *pxtalk_mcps_per_spad)
{

	const FixPoint1616_t cmin_xtalk_per_spad = 8; 
	const FixPoint1616_t ccompensation2 = 13;
	const FixPoint1616_t ccompensation1 = 7; 
	const FixPoint1616_t ctalk_thresh = 66;	 
	const uint32_t c16BitRoundingParam = 0x00008000;
	VL53L0_Error status = VL53L0_ERROR_NONE;
	FixPoint1616_t xtalk_mcps;
	FixPoint1616_t vcsel_width_to_period_ratio;
	FixPoint1616_t integration_time_us;
	uint32_t integration_time_us_int;
	uint8_t vcsel_width_pclk = 3;

	

	if (vcsel_pulse_period_pclk == 0 || timeout_ms == 0)
		status = VL53L0_ERROR_DIVISION_BY_ZERO;


	if (status == VL53L0_ERROR_NONE) {

		
		vcsel_width_to_period_ratio =
			((vcsel_width_pclk << 16) +
			(vcsel_pulse_period_pclk/2))/vcsel_pulse_period_pclk;

		
		integration_time_us = timeout_ms * vcsel_width_to_period_ratio
			* 1000;

		
		integration_time_us_int = (integration_time_us +
			c16BitRoundingParam) >> 16;

		
		xtalk_mcps = rtn_signal_events << 16;
		xtalk_mcps = (xtalk_mcps +
			(integration_time_us_int/2))/integration_time_us_int;

		
		*pxtalk_mcps_per_spad = (xtalk_mcps + (max_spads/2))/max_spads;

		if (*pxtalk_mcps_per_spad < ctalk_thresh)
			*pxtalk_mcps_per_spad = *pxtalk_mcps_per_spad
				- ccompensation2;
		else
			*pxtalk_mcps_per_spad = *pxtalk_mcps_per_spad
				- ccompensation1;

		if (*pxtalk_mcps_per_spad < cmin_xtalk_per_spad)
			*pxtalk_mcps_per_spad = cmin_xtalk_per_spad;

	}
	

	return status;
}


uint32_t bytes_to_int(uint8_t *data_bytes)
{
	uint32_t data = (uint32_t)data_bytes[0] << 24;
	data += ((uint32_t)data_bytes[1] << 16);
	data += ((uint32_t)data_bytes[2] << 8);
	data += ((uint32_t)data_bytes[3]);
	return data;
}

VL53L0_Error perform_histo_signal_meas(VL53L0_DEV dev,
	WindowSelection window_select,
	uint32_t *psignal_events)
{
	VL53L0_Error status = VL53L0_ERROR_NONE;
	uint8_t data[8];
	uint8_t readout_ctrl_val;
	uint32_t bin_width = 3;

	

	if (status == VL53L0_ERROR_NONE) {
		readout_ctrl_val = bin_width;
		if (window_select == VL53L0_AMBIENT_WINDOW_ONLY)
			readout_ctrl_val += 0x80;

		status = VL53L0_WrByte(
			dev, VL53L0_REG_HISTOGRAM_CONFIG_READOUT_CTRL,
			readout_ctrl_val);
	}

	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_start_histogram_measurement(
			dev, VL53L0_HISTOGRAMMODE_RETURN_ONLY, 0);

	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_measurement_poll_for_completion(dev);

	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_ClearInterruptMask(dev, 0);

	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_WrByte(dev, 0xFF, VL53L0_REG_RESULT_CORE_PAGE);


	if (status == VL53L0_ERROR_NONE) {
		status = VL53L0_ReadMulti(dev,
		(uint8_t)VL53L0_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN,
			data,
			8);
	}

	if (status == VL53L0_ERROR_NONE)
		status |= VL53L0_WrByte(dev, 0xFF, 0x00);


	if (status == VL53L0_ERROR_NONE)
		*psignal_events =  bytes_to_int(data) +
			bytes_to_int(&(data[4]));


	

	return status;
}

VL53L0_Error set_final_range_timeout_us(
	VL53L0_DEV dev, uint32_t timeout_microSecs,
	uint16_t final_range_vcsel_period_pclks)
{
	VL53L0_Error status = VL53L0_ERROR_NONE;
	uint16_t final_range_timeout_mclks;
	uint16_t final_range_encoded_timeOut;

	


	
	final_range_timeout_mclks = VL53L0_calc_timeout_mclks(dev,
		timeout_microSecs, (uint8_t) final_range_vcsel_period_pclks);

	
	final_range_encoded_timeOut = VL53L0_encode_timeout(
		final_range_timeout_mclks);

	
	status = VL53L0_WrWord(dev,
		VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
		final_range_encoded_timeOut);

	

	return status;
}


VL53L0_Error perform_histogram_config(VL53L0_DEV dev,
		uint32_t timeout_ms, uint16_t final_range_vcsel_period_pclks)
{
	VL53L0_Error status = VL53L0_ERROR_NONE;
	uint8_t phaseSelect = 1;

	

	if (status == VL53L0_ERROR_NONE)
		status = set_final_range_timeout_us(
			dev, timeout_ms * 1000, final_range_vcsel_period_pclks);

	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_SetDeviceMode(dev,
			VL53L0_DEVICEMODE_SINGLE_HISTOGRAM);


	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_SetHistogramMode(dev,
			VL53L0_HISTOGRAMMODE_BOTH);


	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_WrByte(dev, VL53L0_REG_SYSTEM_HISTOGRAM_BIN,
			0x00);


	
	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_WrByte(dev,
			VL53L0_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT,
			phaseSelect);


	

	return status;
}


VL53L0_Error VL53L0_perform_xtalk_measurement(VL53L0_DEV dev,
	uint32_t timeout_ms, FixPoint1616_t *pxtalk_per_spad,
	uint8_t *pambient_too_high)
{
	VL53L0_Error status = VL53L0_ERROR_NONE;
	uint32_t signal_events = 0;
	uint32_t amb_events = 0;
	uint32_t meas_timing_budget_us;
	VL53L0_DeviceModes device_mode;
	uint8_t final_range_vcsel_period_pclks;
	uint32_t max_spads;

	
	status = VL53L0_GetDeviceMode(dev, &device_mode);

	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_get_max_spads(dev, &max_spads,
			pambient_too_high);

	if (status != VL53L0_ERROR_NONE)
		return status;


	if (status == VL53L0_ERROR_NONE) {
		status = VL53L0_GetVcselPulsePeriod(
			dev,
			VL53L0_VCSEL_PERIOD_FINAL_RANGE,
			&final_range_vcsel_period_pclks);
	}

	if (status == VL53L0_ERROR_NONE) {
		if (final_range_vcsel_period_pclks < 10)
			status = VL53L0_ERROR_INVALID_PARAMS;
	}

	if (status == VL53L0_ERROR_NONE) {
		perform_histogram_config(
			dev, timeout_ms, final_range_vcsel_period_pclks);
	}

	if (status == VL53L0_ERROR_NONE) {
		status = perform_histo_signal_meas(
			dev,
			VL53L0_AMBIENT_WINDOW_ONLY,
			&amb_events);
	}

	if (status == VL53L0_ERROR_NONE) {
		status = perform_histo_signal_meas(
			dev,
			VL53L0_AMBIENT_AND_SIGNAL_WINDOW,
			&signal_events);
	}

	if (status == VL53L0_ERROR_NONE) {
		status = calc_xtalk_mcps_per_spad(
			(signal_events - amb_events),
			timeout_ms,
			max_spads,
			final_range_vcsel_period_pclks,
			pxtalk_per_spad);
	}

	
	if (status == VL53L0_ERROR_NONE)
		status = VL53L0_SetDeviceMode(dev, device_mode);

	if (status == VL53L0_ERROR_NONE) {
		VL53L0_GETPARAMETERFIELD(
			dev,
			MeasurementTimingBudgetMicroSeconds,
			meas_timing_budget_us);

		status = VL53L0_SetMeasurementTimingBudgetMicroSeconds(
			dev, meas_timing_budget_us);
	}

	return status;
}


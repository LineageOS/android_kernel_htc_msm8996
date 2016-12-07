/*
 *Copyright 2014,2015 NXP Semiconductors
 *
 *Licensed under the Apache License, Version 2.0 (the "License");
 *you may not use this file except in compliance with the License.
 *You may obtain a copy of the License at
 *            
 *http://www.apache.org/licenses/LICENSE-2.0
 *             
 *Unless required by applicable law or agreed to in writing, software
 *distributed under the License is distributed on an "AS IS" BASIS,
 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *See the License for the specific language governing permissions and
 *limitations under the License.
 */
 
#include "tfa_dsp_fw.h"
#include "tfa_service.h"
#include "tfa_internal.h"

#include "tfa98xx_tfafieldnames.h"

static enum Tfa98xx_Error tfa9890_specific(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned short regRead = 0;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	

	error = tfa98xx_write_register16(handle, 0x40, 0x5a6b);
	if (error)
		return error;
	tfa98xx_read_register16(handle, 0x59, &regRead);
	regRead |= 0x3;
	tfa98xx_write_register16(handle, 0x59, regRead);
	error = tfa98xx_write_register16(handle, 0x40, 0x0000);

	error = tfa98xx_write_register16(handle, 0x47, 0x7BE1);

	return error;
}

static enum Tfa98xx_Error tfa9890_dsp_system_stable(Tfa98xx_handle_t handle, int *ready)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned short status, mtp0;
	int result, tries;

	
	result = TFA_READ_REG(handle, AREFS);
	if (result < 0) {
		error = -result;
		goto errorExit;
	}
	status = (unsigned short)result;

	*ready = (TFA_GET_BF_VALUE(handle, AMPS, status) == 1);
	if (*ready)		
		return error;	

	
	*ready = !((TFA_GET_BF_VALUE(handle, AREFS, status) == 0)
		   || (TFA_GET_BF_VALUE(handle, CLKS, status) == 0));
	if (!*ready)		
		return error;	

	for (tries = 2; tries > 0; tries--) {
		result = TFA_GET_BF(handle, MTPB);
		if (result < 0) {
			error = -result;
			goto errorExit;
		}
		status = (unsigned short)result;

		
		*ready = (result == 0);
		if (*ready)	
			break;
	}
	if (tries == 0)		
		return Tfa98xx_Error_Ok;


	error = tfa98xx_read_register16(handle, 0x84, &mtp0);
	if (error)
		goto errorExit;

	*ready = (mtp0 != 0);	/* The MTP register written? */

	return error;

errorExit:
	*ready = 0;
	return error;
}

#define TFA98XX_CURRENTSENSE4_CTRL_CLKGATECFOFF (1<<2)
#define TFA98XX_CURRENTSENSE4 0x49
static enum Tfa98xx_Error tfa9890_clockgating(Tfa98xx_handle_t handle, int on)
{
	enum Tfa98xx_Error error;
	unsigned short value;

	
	error = tfa98xx_read_register16(handle, TFA98XX_CURRENTSENSE4, &value);
	if (error) return error;

	if (Tfa98xx_Error_Ok == error) {
		if (on)  
			value &= ~TFA98XX_CURRENTSENSE4_CTRL_CLKGATECFOFF;
		else  
			value |= TFA98XX_CURRENTSENSE4_CTRL_CLKGATECFOFF;

		error = tfa98xx_write_register16(handle, TFA98XX_CURRENTSENSE4, value);
	}

	return error;
}

static enum Tfa98xx_Error tfa9890_dsp_reset(Tfa98xx_handle_t handle, int state)
{
	enum Tfa98xx_Error error;

	tfa9890_clockgating(handle, 0);

        TFA_SET_BF(handle, RST, (uint16_t)state);

	
	error = tfa9890_clockgating(handle, 1);

	return error;
}

void tfa9890_ops(struct tfa_device_ops *ops) {
	ops->tfa_init = tfa9890_specific;
	ops->tfa_dsp_reset = tfa9890_dsp_reset;
	ops->tfa_dsp_system_stable = tfa9890_dsp_system_stable;
}


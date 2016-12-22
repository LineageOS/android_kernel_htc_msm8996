/*
 *Copyright 2015 NXP Semiconductors
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

#include "config.h"

#include "tfa98xx_tfafieldnames.h"
#include "tfa_internal.h"
#include "tfa.h"
#include "tfa_service.h"
#include "tfa_container.h"
#include "tfa_dsp_fw.h"
#include "tfa98xx_genregs_N1C.h"

#define TFA_MK_BF(reg, pos, len) ((reg<<8)|(pos<<4)|(len-1))

#define FAM_TFA98XX_CF_CONTROLS (TFA_FAM(handle,RST) >> 8)
#define FAM_TFA98XX_CF_MEM      (TFA_FAM(handle,MEMA)>> 8)
#define FAM_TFA98XX_MTP0        (TFA_FAM(handle,MTPOTC) >> 8)
#define FAM_TFA98xx_INT_EN      (TFA_FAM(handle,INTENVDDS) >> 8)

#define CF_STATUS_I2C_CMD_ACK 0x01

#if (defined(TFA9888) || defined(TFA98XX_FULL))
void tfa9888_ops(struct tfa_device_ops *ops);
#endif
#if (defined(TFA9891) || defined(TFA98XX_FULL))
void tfa9891_ops(struct tfa_device_ops *ops);
#endif
#if (defined(TFA9897) || defined(TFA98XX_FULL))
void tfa9897_ops(struct tfa_device_ops *ops);
#endif
#if (defined(TFA9890) || defined(TFA98XX_FULL))
void tfa9890_ops(struct tfa_device_ops *ops);
#endif
#if (defined(TFA9887B) || defined(TFA98XX_FULL))
int tfa9887B_is87(Tfa98xx_handle_t handle);
void tfa9887B_ops(struct tfa_device_ops *ops);
#endif
#if (defined(TFA9887) || defined(TFA98XX_FULL))
void tfa9887_ops(struct tfa_device_ops *ops);
#endif

#ifndef MIN
#define MIN(A,B) (A<B?A:B)
#endif

#define CFSTABLE_TRIES   10
#define PWDNWAIT_TRIES   50
#define AMPOFFWAIT_TRIES 50
#define MTPBWAIT_TRIES   50

static int tfa98xx_runtime_verbose;
static int tfa98xx_trace_level;

#define MAX_HANDLES 4
TFA_INTERNAL struct Tfa98xx_handle_private handles_local[MAX_HANDLES];


TFA_INTERNAL int tfa98xx_handle_is_open(Tfa98xx_handle_t h)
{
	int retval = 0;

	if ((h >= 0) && (h < MAX_HANDLES))
		retval = handles_local[h].in_use != 0;

	return retval;
}

int print_calibration(Tfa98xx_handle_t handle, char *str, size_t size)
{
	return snprintf(str, size, " Prim:%d mOhms, Sec:%d mOhms\n",
				handles_local[handle].mohm[0],
				handles_local[handle].mohm[1]);
}

int tfa_get_calibration_info(Tfa98xx_handle_t handle, int channel) {
		return handles_local[handle].mohm[channel];
}

static void tfa_set_query_info (int dev_idx) {

	if (dev_idx > MAX_HANDLES) {
		_ASSERT(dev_idx > MAX_HANDLES);
		return;
	}
	
	
	handles_local[dev_idx].hw_feature_bits = -1;
	handles_local[dev_idx].sw_feature_bits[0] = -1;
	handles_local[dev_idx].sw_feature_bits[1] = -1;
	handles_local[dev_idx].profile = -1;
	handles_local[dev_idx].vstep[0] = -1;
	handles_local[dev_idx].vstep[1] = -1;
	
	handles_local[dev_idx].tfa_family = 1;
	handles_local[dev_idx].daimap = Tfa98xx_DAI_I2S;		
	handles_local[dev_idx].spkr_count = 1;
	handles_local[dev_idx].spkr_select = 0;
	handles_local[dev_idx].support_tcoef = supportYes;
	handles_local[dev_idx].supportDrc = supportNotSet;
	handles_local[dev_idx].support_saam = supportNotSet;
         

	switch (handles_local[dev_idx].rev & 0xff) {
	case 0x88:
		
		handles_local[dev_idx].tfa_family = 2;
		handles_local[dev_idx].spkr_count = 2;
		handles_local[dev_idx].daimap = Tfa98xx_DAI_TDM;
		tfa9888_ops(&handles_local[dev_idx].dev_ops); 
		break;
	case 0x97:
		
		handles_local[dev_idx].supportDrc = supportNo;
		handles_local[dev_idx].spkr_count = 1;
		handles_local[dev_idx].daimap = Tfa98xx_DAI_TDM;
		tfa9897_ops(&handles_local[dev_idx].dev_ops); 
		break;
	case 0x92:
		
		handles_local[dev_idx].spkr_count = 1;
		handles_local[dev_idx].daimap = ( Tfa98xx_DAI_PDM | Tfa98xx_DAI_I2S );
		tfa9891_ops(&handles_local[dev_idx].dev_ops); 
		break;
	case 0x91:
		
		handles_local[dev_idx].spkr_count = 1;
		handles_local[dev_idx].daimap = ( Tfa98xx_DAI_PDM | Tfa98xx_DAI_I2S );
		break;
	case 0x80:
	case 0x81:
		
		handles_local[dev_idx].spkr_count = 1;
		handles_local[dev_idx].daimap = Tfa98xx_DAI_I2S;
		handles_local[dev_idx].supportDrc = supportNo;
		handles_local[dev_idx].supportFramework = supportNo;
		tfa9890_ops(&handles_local[dev_idx].dev_ops); 
		break;
	case 0x12:
		
		handles_local[dev_idx].spkr_count = 1;
		handles_local[dev_idx].daimap = Tfa98xx_DAI_I2S;
		if (tfa9887B_is87(dev_idx)) {
			handles_local[dev_idx].support_tcoef = supportNo;
			tfa9887_ops(&handles_local[dev_idx].dev_ops); 
		} else
			tfa9887B_ops(&handles_local[dev_idx].dev_ops); 
		break;
	default:
		pr_err("unknown device type : 0x%02x\n", handles_local[dev_idx].rev);
		_ASSERT(0);
		break;
	}
}

int tfa98xx_dev2family(int dev_type) {
	
	switch(dev_type & 0xff) {
	case 0x12:
	case 0x80:
	case 0x81:
	case 0x91:
	case 0x92:
	case 0x97:
		return 1;
	case 0x88:
		return 2;
	case 0x50:
		return 3;
	default:
		return 0;
	}
}

int tfa98xx_dev_family(Tfa98xx_handle_t dev_idx) {
	return handles_local[dev_idx].tfa_family;
}

unsigned short tfa98xx_dev_revision(Tfa98xx_handle_t dev_idx)
{
	return handles_local[dev_idx].rev;
}

void tfa98xx_set_spkr_select(Tfa98xx_handle_t dev_idx, char *configuration) {
	 char firstLetter;

	
	if (configuration==NULL)
		handles_local[dev_idx].spkr_select = 0;
	else
	{
		firstLetter = (char)tolower((unsigned char)configuration[0]);
		switch (firstLetter) {
		        case 'b': 
				handles_local[dev_idx].spkr_select = 8;
				handles_local[dev_idx].spkr_count = 2;
				break;
			case 'l':
			case 'p': 
				handles_local[dev_idx].spkr_select = 4;
				handles_local[dev_idx].spkr_count = 1;
				break;
			case 'r':
			case 's': 
				handles_local[dev_idx].spkr_select = 2;
				handles_local[dev_idx].spkr_count = 1;
				break;
			case 'd': 
				handles_local[dev_idx].spkr_select = 1;
				handles_local[dev_idx].spkr_count = 2;
				break;
			default:
				handles_local[dev_idx].spkr_select = 0;
				handles_local[dev_idx].spkr_count = 2;
				break;
		}
	}
}

void tfa_mock_probe(int dev_idx, unsigned short revid, int slave_address)
{
        handles_local[dev_idx].slave_address = (unsigned char)slave_address*2;
        handles_local[dev_idx].rev = revid;
        tfa_set_query_info(dev_idx);
}

enum Tfa98xx_Error
tfa_soft_probe(int dev_idx, int revid)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	error = tfaContGetSlave(dev_idx, &handles_local[dev_idx].slave_address);
	handles_local[dev_idx].slave_address *=2;
	if (error)
		return error;

	handles_local[dev_idx].rev = (unsigned short)revid;
	tfa_set_query_info(dev_idx);

	return error;
}

enum Tfa98xx_Error
tfa_probe(unsigned char slave_address, Tfa98xx_handle_t *pHandle)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int rev;
	int idx;

	_ASSERT(pHandle != NULL);
	*pHandle = -1;

	
	idx = tfa98xx_cnt_slave2idx(slave_address>>1);
	if (idx < 0)
		idx = 0; 

	if (handles_local[idx].in_use == 1)
		return Tfa98xx_Error_InUse;

	handles_local[idx].in_use = 1;

	switch (slave_address) {
	case TFA98XX_GENERIC_SLAVE_ADDRESS:     
	case 0x68:
	case 0x6A:
	case 0x6C:
	case 0x6E:
	case (0x1a<<1): 
		handles_local[idx].buffer_size = MAX_I2C_BUFFER_SIZE;
		handles_local[idx].slave_address = slave_address;
#if (defined(TFA9887)  || defined (TFA9887B) || defined(TFA98XX_FULL))
		rev = TFA_READ_REG(idx, REV);
#endif
		
		rev = TFA_READ_REG(idx, REV);
		if (rev < 0) 
			error = -rev;
		if (Tfa98xx_Error_Ok != error) {
			handles_local[idx].in_use = 0;
			pr_debug("\nError: Unable to read revid from slave:0x%02x \n", slave_address/2);
			return error;
		}
		handles_local[idx].rev = (unsigned short) rev;
		*pHandle = idx;
		error = Tfa98xx_Error_Ok;
#ifdef __KERNEL__ 
		tfa98xx_trace_printk("slave:0x%02x revid:0x%04x\n", slave_address, rev);
		pr_debug("slave:0x%02x revid:0x%04x\n", slave_address, rev);
#endif
		break;
	default:
		
		error = Tfa98xx_Error_Bad_Parameter;
	}

	tfa_set_query_info(idx);

	handles_local[idx].in_use = 0;

	return error;
}

enum Tfa98xx_Error
tfa98xx_open(Tfa98xx_handle_t handle)
{
	if (tfa98xx_handle_is_open(handle)) {
		return Tfa98xx_Error_InUse;
	} else {
		handles_local[handle].in_use = 1;
		return Tfa98xx_Error_Ok;
	}
}

enum Tfa98xx_Error tfa98xx_close(Tfa98xx_handle_t handle)
{
	if (tfa98xx_handle_is_open(handle)) {
		handles_local[handle].in_use = 0;
		return Tfa98xx_Error_Ok;
	} else {
		return Tfa98xx_Error_NotOpen;
	}
}

enum Tfa98xx_DMEM tfa98xx_filter_mem(Tfa98xx_handle_t dev, int filter_index, unsigned short *address, int channel) 
{
	enum Tfa98xx_DMEM  dmem=-1;
	int idx;
	unsigned short bq_table[7][4] ={
	
			{346,351,356,288}, 
			{346,351,356,288}, 
			{467,472,477,409}, 
			{406,411,416,348}, 
			{467,472,477,409}, 
			{8832, 8837, 8842, 8847}, 
			{8853, 8858, 8863, 8868}  
	};

	if ( (10 <= filter_index) && (filter_index <= 13) ) {
		dmem = Tfa98xx_DMEM_YMEM; 
		idx = filter_index-10;

		switch (handles_local[dev].rev & 0xff ) { 
		case 0x12:
			if ( tfa9887B_is87(dev) )
	        		*address = bq_table[0][idx];
			else
				*address = bq_table[2][idx];
			break;
		case 0x97:
			*address = bq_table[3][idx];
			break;
		case 0x80:
		case 0x81: 
		case 0x91:
			*address = bq_table[1][idx];
			break;
		case 0x92:
			*address = bq_table[4][idx];
			break;
		case 0x88:
			
			if(channel == 1)
				*address = bq_table[5][idx];
			else
				*address = bq_table[6][idx];
			break;
		default:
			
			return -1;
			_ASSERT(0);
		}
	}
	return dmem;
}

void tfa98xx_rev(int *major, int *minor, int *revision) {
	*major = TFA98XX_API_REV_MAJOR;
	*minor = TFA98XX_API_REV_MINOR;
    *revision = TFA98XX_API_REV_REVISION;
}

int tfa98xx_max_devices(void) 
{
	return MAX_HANDLES;
}

unsigned short tfa98xx_get_device_revision(Tfa98xx_handle_t handle)
{
	
	return handles_local[handle].rev;
}

enum Tfa98xx_DAI tfa98xx_get_device_dai(Tfa98xx_handle_t handle)
{
	
	return handles_local[handle].daimap;
}

enum Tfa98xx_Error tfa98xx_dsp_support_tcoef(Tfa98xx_handle_t dev_idx,
		int *pb_support_tCoef)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	
	if ( handles_local[dev_idx].support_tcoef != supportNotSet) {
		*pb_support_tCoef = (handles_local[dev_idx].support_tcoef == supportYes);
	}

	handles_local[dev_idx].support_tcoef = *pb_support_tCoef ? supportYes : supportNo;

	return error;
}

enum Tfa98xx_Error tfa98xx_supported_speakers(Tfa98xx_handle_t handle, int* spkr_count)
{
	if (tfa98xx_handle_is_open(handle)) {
		*spkr_count = handles_local[handle].spkr_count;
	} else
		return Tfa98xx_Error_NotOpen;

	return Tfa98xx_Error_Ok;
}

enum Tfa98xx_Error tfa98xx_supported_dai(Tfa98xx_handle_t handle, enum Tfa98xx_DAI *daimap)
{
	if (tfa98xx_handle_is_open(handle)) {
		*daimap = handles_local[handle].daimap;
	} else
		return Tfa98xx_Error_NotOpen;

	return Tfa98xx_Error_Ok;
}

enum Tfa98xx_Error tfa98xx_supported_saam(Tfa98xx_handle_t handle, enum Tfa98xx_saam *saam)
{
	int features;
	enum Tfa98xx_Error error;

	if (handles_local[handle].support_saam == supportNotSet) {
		error = tfa98xx_dsp_get_hw_feature_bits(handle,&features);
		if (error!=Tfa98xx_Error_Ok)
			return error;
		handles_local[handle].support_saam =
				(features & 0x8000)? supportYes : supportNo; 
	}
	*saam = handles_local[handle].support_saam == supportYes ? Tfa98xx_saam : Tfa98xx_saam_none ;

	return Tfa98xx_Error_Ok;
}

enum Tfa98xx_Error tfa98xx_compare_features(Tfa98xx_handle_t handle, int features_from_MTP[3], int features_from_cnt[3])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
        uint32_t value;
	uint16_t mtpbf;
        unsigned char bytes[3 * 2];

        
	

        
        int status;
	tfa98xx_dsp_system_stable(handle, &status);
	if (!status)
                return Tfa98xx_Error_NoClock; 

        
        if (tfa98xx_dev_family(handle) == 1) {
	        mtpbf=0x850f;  
	} else {
		mtpbf=0xf907;  
        }

        
        value = tfa_read_reg(handle, mtpbf) & 0xffff;
	features_from_MTP[0] = handles_local[handle].hw_feature_bits = value;

        
        error = tfa_dsp_cmd_id_write_read(handle, MODULE_FRAMEWORK, 
                FW_PAR_ID_GET_FEATURE_INFO, sizeof(bytes), bytes);
	if (error != Tfa98xx_Error_Ok) 
	        return error; 
        tfa98xx_convert_bytes2data(sizeof(bytes), bytes, &features_from_MTP[1]);

        
        get_hw_features_from_cnt(handle, &features_from_cnt[0]);
        get_sw_features_from_cnt(handle, &features_from_cnt[1]);

	return error;
}

enum Tfa98xx_Error tfa98xx_dsp_reset(Tfa98xx_handle_t dev_idx, int state)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	if (tfa98xx_handle_is_open(dev_idx)) {
		if (handles_local[dev_idx].dev_ops.tfa_dsp_reset)
			error = (*handles_local[dev_idx].dev_ops.tfa_dsp_reset)(dev_idx, state);
		else
			
			TFA_SET_BF_VOLATILE(dev_idx, RST, (uint16_t)state);
	}
	return error;
}


static enum Tfa98xx_Error _tfa98xx_dsp_system_stable(Tfa98xx_handle_t handle,
						int *ready)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned short status;
	int value;

	
	value = TFA_READ_REG(handle, AREFS);
	if (value < 0) {
		error = -value;
		*ready = 0;
		_ASSERT(error);		
		return error;
	}
	status = (unsigned short)value;

	
	*ready = !((TFA_GET_BF_VALUE(handle, AREFS, status) == 0)
		   || (TFA_GET_BF_VALUE(handle, CLKS, status) == 0));

	return error;
}

void tfa98xx_apply_deferred_calibration(Tfa98xx_handle_t handle)
{
	struct tfa98xx_controls *controls = &(handles_local[handle].dev_ops.controls);
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	unsigned short value;

	if (controls->otc.deferrable && controls->otc.triggered) {
		pr_debug("Deferred writing otc = %d\n", controls->otc.wr_value);
		err = tfa98xx_set_mtp(handle,
			(uint16_t)controls->otc.wr_value << TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_POS,
			1 << TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_POS);
		if (err != Tfa98xx_Error_Ok) {
			pr_err("Unable to apply deferred MTP OTC write. Error=%d\n",
									err);
		} else {
			controls->otc.triggered = false;
			controls->otc.rd_valid = true;
			err = tfa98xx_get_mtp(handle, &value);
			if (err == Tfa98xx_Error_Ok)
				controls->otc.rd_value =
					(value & TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_MSK)
					>> TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_POS;
			else
				controls->otc.rd_value = controls->otc.wr_value;
		}
	}

	if (controls->mtpex.deferrable && controls->mtpex.triggered) {
		pr_debug("Deferred writing mtpex = %d\n", controls->mtpex.wr_value);
		err = tfa98xx_set_mtp(handle,
			(uint16_t)controls->mtpex.wr_value << TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_POS,
			1 << TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_POS);
		if (err != Tfa98xx_Error_Ok) {
			pr_err("Unable to apply deferred MTPEX write. Rrror=%d\n",
									err);
		} else {
			controls->mtpex.triggered = false;
			controls->mtpex.rd_valid = true;
			err = tfa98xx_get_mtp(handle, &value);
			if (err == Tfa98xx_Error_Ok)
				controls->otc.rd_value =
					(value & TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_MSK)
					>> TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_POS;
			else
				controls->mtpex.rd_value = controls->mtpex.wr_value;
		}
	}

	if (controls->calib.triggered) {
		err = tfa_calibrate(handle);
		if(err) {
			pr_info("Deferred calibration failed: %d\n", err);
		} else {
			pr_debug("Deferred calibration ok\n");
			controls->calib.triggered = false;
		}
	}
}

enum Tfa98xx_Error tfa98xx_dsp_system_stable(Tfa98xx_handle_t dev_idx, int *ready)
{
	enum Tfa98xx_Error error;

	if (!tfa98xx_handle_is_open(dev_idx))
		return Tfa98xx_Error_NotOpen;

	if (handles_local[dev_idx].dev_ops.tfa_dsp_system_stable)
		error = (*handles_local[dev_idx].dev_ops.tfa_dsp_system_stable)(dev_idx, ready);
	else
		
		error = _tfa98xx_dsp_system_stable(dev_idx, ready);
	return error;
}

int tfa98xx_cf_enabled(Tfa98xx_handle_t dev_idx)
{
	if (!tfa98xx_handle_is_open(dev_idx))
		return Tfa98xx_Error_NotOpen;

	return TFA_GET_BF(dev_idx, CFE);
}


enum Tfa98xx_Error tfa98xx_init(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	uint16_t value=0;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	TFA_SET_BF_VALUE(handle, I2CR, 1, &value );
	TFA_WRITE_REG(handle, I2CR, value);

	if (tfa98xx_dev_family(handle) == 2) {
		
		TFA_SET_BF_VOLATILE(handle, MANSCONF, 0);
		TFA_SET_BF_VOLATILE(handle, MANCOLD, 1);
	} else {
		handles_local[handle].dev_ops.controls.otc.deferrable = true;
		handles_local[handle].dev_ops.controls.mtpex.deferrable = true;
	}
	
	tfa98xx_dsp_reset(handle, 1); 



	if (handles_local[handle].dev_ops.tfa_init)
		error = (*handles_local[handle].dev_ops.tfa_init)(handle);
	

	return error;
}

enum Tfa98xx_Error tfa98xx_dsp_write_tables(Tfa98xx_handle_t handle, int sample_rate)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	if (handles_local[handle].dev_ops.tfa_dsp_write_tables)
		error = (*handles_local[handle].dev_ops.tfa_dsp_write_tables)(handle, sample_rate);

	return error;
}

enum Tfa98xx_Error tfa98xx_dsp_get_memory(Tfa98xx_handle_t handle, int memoryType, 
		int offset, int length, unsigned char bytes[])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	char msg[12];

	msg[0] = 8;
	msg[1] = MODULE_FRAMEWORK + 128;
	msg[2] = FW_PAR_ID_GET_MEMORY;

	msg[3] = 0;
	msg[4] = 0;
	msg[5] = (char)memoryType;

	msg[6] = 0;
	msg[7] = (offset>>8) & 0xff;
	msg[8] = offset & 0xff;

	msg[9] = 0;
	msg[10] = (length>>8) & 0xff;
	msg[11] = length & 0xff;

	
	error = tfa_dsp_msg(handle, sizeof(msg), (char *)msg);

	if (error != Tfa98xx_Error_Ok)
		return error;

	
	error = tfa_dsp_msg_read(handle, length * 3, bytes);	

	return error;
}

enum Tfa98xx_Error tfa98xx_dsp_set_memory(Tfa98xx_handle_t handle, int memoryType, 
		int offset, int length, int value)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	char msg[15];

	msg[0] = 8;
	msg[1] = MODULE_FRAMEWORK + 128;
	msg[2] = FW_PAR_ID_SET_MEMORY;

	msg[3] = 0;
	msg[4] = 0;
	msg[5] = (char)memoryType;

	msg[6] = 0;
	msg[7] = (offset>>8) & 0xff;
	msg[8] = offset & 0xff;

	msg[9] = 0;
	msg[10] = (length>>8) & 0xff;
	msg[11] = length & 0xff;

	msg[12] = (value>>16) & 0xff;
	msg[13] = (value>>8) & 0xff;
	msg[14] = value & 0xff;

	
	error = tfa_dsp_msg(handle, sizeof(msg), (char *)msg);

	return error;
}
enum Tfa98xx_Error tfa98xx_get_mtp(Tfa98xx_handle_t handle, uint16_t *value)
{
	int status;
	int result;

	
	if ( TFA_GET_BF(handle, PWDN) ) {
		pr_debug("PLL in powerdown\n");
		return Tfa98xx_Error_NoClock;
	}

	
	if (tfa98xx_dev_family(handle) == 1) {
		tfa98xx_dsp_system_stable(handle, &status);
		if (status==0) {
			pr_debug("PLL not running\n");
			return Tfa98xx_Error_NoClock;
		}
	}

	result = TFA_READ_REG(handle, MTP0);
	if (result <  0) {
		return -result;
	}
	*value = (uint16_t)result;

	return Tfa98xx_Error_Ok;
}

void tfa98xx_key2(Tfa98xx_handle_t handle, int lock) {

	
	tfa98xx_write_register16(handle,
				(tfa98xx_dev_family(handle) == 1) ? 0x40 :0x0F, 0x5A6B);
	
	TFA_WRITE_REG(handle, MTPKEY2, lock? 0 :0x5A );
	
	tfa98xx_write_register16(handle,
				(tfa98xx_dev_family(handle) == 1) ? 0x40 :0x0F, 0);


}

enum Tfa98xx_Error tfa98xx_set_mtp(Tfa98xx_handle_t handle, uint16_t value,
		uint16_t mask)
{
	unsigned short mtp_old, mtp_new;
	int loop, status;
	enum Tfa98xx_Error error;

	error = tfa98xx_get_mtp(handle, &mtp_old);

	if (error != Tfa98xx_Error_Ok)
		return error;

	mtp_new = (value & mask) | (mtp_old & ~mask);

	if ( mtp_old == mtp_new) 
		return Tfa98xx_Error_Ok;

	
	error = tfa98xx_dsp_system_stable(handle, &status);
	if (error)
		return error;
	if (status==0)
		return Tfa98xx_Error_NoClock;

	tfa98xx_key2(handle , 0); 
	TFA_WRITE_REG(handle, MTP0, mtp_new); 	
	
	TFA_SET_BF(handle, CIMTP, 1);
	
	tfa98xx_key2(handle , 1); 

	for(loop=0; loop<100  ;loop++) {
		msleep_interruptible(10); 			
		if (TFA_GET_BF(handle, MTPB) == 0)
			return Tfa98xx_Error_Ok;
	}

	return Tfa98xx_Error_StateTimedOut;
}
int tfa_calibrate(Tfa98xx_handle_t handle) {
	enum Tfa98xx_Error error;

	
	error = tfa98xx_set_mtp(handle, 0, 0x2);
	if (error)
		return error ;

	
	error = tfaRunColdboot(handle, 1);

	
	return error;
}

static short twos(short x)
{
	 return (x<0)? x+512 : x;
}
void tfa98xx_set_exttemp(Tfa98xx_handle_t handle, short ext_temp)
{
	if ((-256 <= ext_temp) && (ext_temp <= 255)) {
		
		pr_debug("Using ext temp %d C\n", twos(ext_temp));
		TFA_SET_BF(handle, TROS, 1);
		TFA_SET_BF(handle, EXTTS, twos(ext_temp));
	} else {
		pr_debug("Clearing ext temp settings\n");
		TFA_SET_BF(handle, TROS, 0);
	}
}
short tfa98xx_get_exttemp(Tfa98xx_handle_t handle)
{
	short ext_temp = (short)TFA_GET_BF(handle, EXTTS);
	return (twos(ext_temp));
}

enum Tfa98xx_Error tfa98xx_set_volume_level(Tfa98xx_handle_t handle, unsigned short vol)
{
	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	if (vol > 255)	
		vol = 255;


	
	return -TFA_SET_BF(handle, VOL, (uint16_t)vol);
}

static enum Tfa98xx_Error
tfa98xx_set_mute_tfa2(Tfa98xx_handle_t handle, enum Tfa98xx_Mute mute)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	switch (mute) {
	case Tfa98xx_Mute_Off:
		TFA_SET_BF(handle, CFSMR, 0);
		TFA_SET_BF(handle, CFSML, 0);
		break;
	case Tfa98xx_Mute_Amplifier:
	case Tfa98xx_Mute_Digital:
		TFA_SET_BF(handle, CFSMR, 1);
		TFA_SET_BF(handle, CFSML, 1);
		break;
	default:
		return Tfa98xx_Error_Bad_Parameter;
	}

	return error;
}

static enum Tfa98xx_Error
tfa98xx_set_mute_tfa1(Tfa98xx_handle_t handle, enum Tfa98xx_Mute mute)
{
	enum Tfa98xx_Error error;
	unsigned short audioctrl_value;
	unsigned short sysctrl_value;
	int value;

	value = TFA_READ_REG(handle, CFSM); 
	if (value < 0)
		return -value;
	audioctrl_value = (unsigned short)value;
	value = TFA_READ_REG(handle, AMPE); 
	if (value < 0)
		return -value;
	sysctrl_value = (unsigned short)value;

	switch (mute) {
	case Tfa98xx_Mute_Off:
		TFA_SET_BF_VALUE(handle, CFSM, 0, &audioctrl_value);
		TFA_SET_BF_VALUE(handle, AMPE, 1, &sysctrl_value);
		TFA_SET_BF_VALUE(handle, DCA, 1, &sysctrl_value);
		break;
	case Tfa98xx_Mute_Digital:
		
		
		TFA_SET_BF_VALUE(handle, CFSM, 1, &audioctrl_value);
		
		TFA_SET_BF_VALUE(handle, AMPE, 1, &sysctrl_value);
		
		TFA_SET_BF_VALUE(handle, DCA, 0, &sysctrl_value);
		break;
	case Tfa98xx_Mute_Amplifier:
		
		TFA_SET_BF_VALUE(handle, CFSM, 0, &audioctrl_value);
		
		TFA_SET_BF_VALUE(handle, AMPE, 0, &sysctrl_value);
		TFA_SET_BF_VALUE(handle, DCA, 0, &sysctrl_value);
		break;
	default:
		return Tfa98xx_Error_Bad_Parameter;
	}

	error = -TFA_WRITE_REG(handle, CFSM, audioctrl_value);
	if (error)
		return error;
	error = -TFA_WRITE_REG(handle, AMPE, sysctrl_value);
	return error;
}

enum Tfa98xx_Error
tfa98xx_set_mute(Tfa98xx_handle_t handle, enum Tfa98xx_Mute mute)
{
	if (!tfa98xx_handle_is_open(handle)) {
		pr_err("device not opened\n");
		return Tfa98xx_Error_NotOpen;
	}

	if (tfa98xx_dev_family(handle) == 1)
		return tfa98xx_set_mute_tfa1(handle, mute);
	else
		return tfa98xx_set_mute_tfa2(handle, mute);
}

static enum Tfa98xx_Error
tfa98xx_process_patch_file(Tfa98xx_handle_t handle, int length,
		 const unsigned char *bytes)
{
	unsigned short size;
	int index = 0;
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	while (index < length) {
		size = bytes[index] + bytes[index + 1] * 256;
		index += 2;
		if ((index + size) > length) {
			
			return Tfa98xx_Error_Bad_Parameter;
		}

		if (size > handles_local[handle].buffer_size) {
			
			return Tfa98xx_Error_Bad_Parameter;
		}

		error = tfa98xx_write_raw(handle, size, &bytes[index]);
		if (error != Tfa98xx_Error_Ok)
			break;
		index += size;
	}
	return  error;
}



static enum Tfa98xx_Error
tfa98xx_check_ic_rom_version(Tfa98xx_handle_t handle, const unsigned char patchheader[])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned short checkrev, revid;
	unsigned char lsb_revid;
	unsigned short checkaddress;
	int checkvalue;
	int value = 0;
	int status;
	checkrev = patchheader[0];
	lsb_revid = handles_local[handle].rev & 0xff; 

	if ((checkrev != 0xFF) && (checkrev != lsb_revid))
		return Tfa98xx_Error_Not_Supported;

	checkaddress = (patchheader[1] << 8) + patchheader[2];
	checkvalue =
	    (patchheader[3] << 16) + (patchheader[4] << 8) + patchheader[5];
	if (checkaddress != 0xFFFF) {
		
		error = tfa98xx_dsp_system_stable(handle, &status);
		if (error == Tfa98xx_Error_Ok) {
			if (!status) {
				
				error = Tfa98xx_Error_DSP_not_running;
			}
		}
		
		if (error == Tfa98xx_Error_Ok) {
			error =
			tfa98xx_dsp_read_mem(handle, checkaddress, 1, &value);
		}
		if (error == Tfa98xx_Error_Ok) {
			if (value != checkvalue) {
				pr_err("patch file romid type check failed [0x%04x]: expected 0x%02x, actual 0x%02x\n",
						checkaddress, value, checkvalue);
				error = Tfa98xx_Error_Not_Supported;
			}
		}
	} else { 
		
		if ( checkvalue != 0xFFFFFF && checkvalue != 0) {
			revid = patchheader[5]<<8 | patchheader[0]; 
			if ( revid != handles_local[handle].rev) {
				pr_err("patch file device type check failed: expected 0x%02x, actual 0x%02x\n",
						handles_local[handle].rev, revid);
				return Tfa98xx_Error_Not_Supported;
			}
		}
	}

	return error;
}


#define PATCH_HEADER_LENGTH 6
enum Tfa98xx_Error
tfa_dsp_patch(Tfa98xx_handle_t handle, int patchLength,
		 const unsigned char *patchBytes)
{
	enum Tfa98xx_Error error;
	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;
	if (patchLength < PATCH_HEADER_LENGTH)
		return Tfa98xx_Error_Bad_Parameter;
	error = tfa98xx_check_ic_rom_version(handle, patchBytes);
	if (Tfa98xx_Error_Ok != error) {
		return error;
	}
	error =
	    tfa98xx_process_patch_file(handle, patchLength - PATCH_HEADER_LENGTH,
			     patchBytes + PATCH_HEADER_LENGTH);
	return error;
}


TFA_INTERNAL enum Tfa98xx_Error
tfa98xx_wait_result(Tfa98xx_handle_t handle, int wait_retry_count)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int cf_status; 
	int tries = 0;
	do {
		cf_status = TFA_GET_BF(handle, ACK);
		if (cf_status < 0)
			error = -cf_status;
		tries++;
	}
	
	
	while ((error == Tfa98xx_Error_Ok) && ((cf_status & CF_STATUS_I2C_CMD_ACK) == 0)
			&& (tries < wait_retry_count));
	if (tries >= wait_retry_count) {
		
		error = Tfa98xx_Error_DSP_not_running;
	}
	return error;
}

void tfa98xx_convert_bytes2data(int num_bytes, const unsigned char bytes[],
			       int data[])
{
	int i;			
	int k;			
	int d;
	int num_data = num_bytes / 3;
	_ASSERT((num_bytes % 3) == 0);
	for (i = 0, k = 0; i < num_data; ++i, k += 3) {
		d = (bytes[k] << 16) | (bytes[k + 1] << 8) | (bytes[k + 2]);
		_ASSERT(d >= 0);
		_ASSERT(d < (1 << 24));	
		if (bytes[k] & 0x80)	
			d = -((1 << 24) - d);

		data[i] = d;
	}
}

void tfa98xx_convert_data2bytes(int num_data, const int data[],
			       unsigned char bytes[])
{
	int i;			
	int k;			
	int d;
	for (i = 0, k = 0; i < num_data; ++i, k += 3) {
		if (data[i] >= 0)
			d = MIN(data[i], (1 << 23) - 1);
		else {
			
			d = (1 << 24) - MIN(-data[i], 1 << 23);
		}
		_ASSERT(d >= 0);
		_ASSERT(d < (1 << 24));	
		bytes[k] = (d >> 16) & 0xFF;	
		bytes[k + 1] = (d >> 8) & 0xFF;
		bytes[k + 2] = (d) & 0xFF;	
	}
}




enum Tfa98xx_Error tfa_dsp_msg_write(Tfa98xx_handle_t handle, int length, const char *buffer)
{
        int offset = 0;
	int chunk_size = ROUND_DOWN(handles_local[handle].buffer_size, 3);  
	int remaining_bytes = length;
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	uint16_t cfctl;
	int value;

	value = TFA_READ_REG(handle, DMEM);
	if (value < 0) {
		error = -value;
		return error;
	}
	cfctl = (uint16_t)value;
	

	TFA_SET_BF_VALUE(handle, DMEM, (uint16_t)Tfa98xx_DMEM_XMEM, &cfctl); 
	TFA_SET_BF_VALUE(handle, AIF, 0, &cfctl ); 
	TFA_WRITE_REG(handle, DMEM, cfctl);

	TFA_WRITE_REG(handle, MADD, 1);

	while ((error == Tfa98xx_Error_Ok) && (remaining_bytes > 0)) {
		if (remaining_bytes < chunk_size)
			chunk_size = remaining_bytes;
		
		error = tfa98xx_write_data(handle, FAM_TFA98XX_CF_MEM,
				      chunk_size, (const unsigned char *)buffer + offset);
		remaining_bytes -= chunk_size;
		offset += chunk_size;
	}

	
	if (error == Tfa98xx_Error_Ok) {
		
		
		TFA_SET_BF_VALUE(handle, REQCMD, 0x01, &cfctl ); 
		TFA_SET_BF_VALUE(handle, CFINT, 1, &cfctl );
		error = -TFA_WRITE_REG(handle, CFINT, cfctl);
	}

	return error;
}

enum Tfa98xx_Error tfa_dsp_msg_write_id(Tfa98xx_handle_t handle, int length, const char *buffer, uint8_t cmdid[3])
{
        int offset = 0;
	int chunk_size = ROUND_DOWN(handles_local[handle].buffer_size, 3);  
	int remaining_bytes = length;
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	uint16_t cfctl;
	int value;

	value = TFA_READ_REG(handle, DMEM);
	if (value < 0) {
		error = -value;
		return error;
	}
	cfctl = (uint16_t)value;
	

	TFA_SET_BF_VALUE(handle, DMEM, (uint16_t)Tfa98xx_DMEM_XMEM, &cfctl); 
	TFA_SET_BF_VALUE(handle, AIF, 0, &cfctl ); 
	TFA_WRITE_REG(handle, DMEM, cfctl);

	TFA_WRITE_REG(handle, MADD, 1);

	
	error = tfa98xx_write_data(handle, FAM_TFA98XX_CF_MEM, 3, (const unsigned char *)cmdid);

	while ((error == Tfa98xx_Error_Ok) && (remaining_bytes > 0)) {
		if (remaining_bytes < chunk_size)
			chunk_size = remaining_bytes;
		
		error = tfa98xx_write_data(handle, FAM_TFA98XX_CF_MEM,
				      chunk_size, (const unsigned char *)buffer + offset);
		remaining_bytes -= chunk_size;
		offset += chunk_size;
	}

	
	if (error == Tfa98xx_Error_Ok) {
		
		
		TFA_SET_BF_VALUE(handle, REQCMD, 0x01, &cfctl ); 
		TFA_SET_BF_VALUE(handle, CFINT, 1, &cfctl );
		error = -TFA_WRITE_REG(handle, CFINT, cfctl);
	}

	return error;
}

enum Tfa98xx_Error tfa_dsp_msg_status(Tfa98xx_handle_t handle, int *pRpcStatus) {
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	error = tfa98xx_wait_result(handle, 2); 
	if (error == Tfa98xx_Error_DSP_not_running) {
		*pRpcStatus = -1;
		return Tfa98xx_Error_Ok;
	}
	else if (error != Tfa98xx_Error_Ok)
		return error;

	error = tfa98xx_check_rpc_status(handle, pRpcStatus);

	return error;
}

const char* tfa98xx_get_i2c_status_id_string(int status)
{
        const char* p_id_str;
        char latest_errorstr[64];

        switch (status)
        {
                case Tfa98xx_DSP_Not_Running:
                        p_id_str = "No response from DSP";
                        break;
                case Tfa98xx_I2C_Req_Done:
                        p_id_str = "Ok";
                        break;
                case Tfa98xx_I2C_Req_Busy:
                        p_id_str = "Request is being processed";
                        break;
                case Tfa98xx_I2C_Req_Invalid_M_ID:
                        p_id_str = "Provided M-ID does not fit in valid rang [0..2]";
                        break;
                case Tfa98xx_I2C_Req_Invalid_P_ID:
                        p_id_str = "Provided P-ID is not valid in the given M-ID context";
                        break;
                case Tfa98xx_I2C_Req_Invalid_CC:
                        p_id_str = "Invalid channel configuration bits (SC|DS|DP|DC) combination";
                        break;
                case Tfa98xx_I2C_Req_Invalid_Seq:
                        p_id_str = "Invalid sequence of commands, in case the DSP expects some commands in a specific order";
                        break;
                case Tfa98xx_I2C_Req_Invalid_Param:
                        p_id_str = "Generic error";
                        break;
                case Tfa98xx_I2C_Req_Buffer_Overflow:
                        p_id_str = "I2C buffer has overflowed: host has sent too many parameters, memory integrity is not guaranteed";
                        break;
                default:
                        sprintf(latest_errorstr, "Unspecified error (%d)", (int)status);
                        p_id_str = latest_errorstr;
                        break;
        }

        return p_id_str;
}

enum Tfa98xx_Error tfa_dsp_msg_read(Tfa98xx_handle_t handle,int length, unsigned char *bytes){
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int burst_size;		
	int bytes_per_word = 3;
	int num_bytes;
	int offset = 0;
	unsigned short start_offset=2; 

	if ( length > TFA2_MAX_PARAM_SIZE)
		return Tfa98xx_Error_Bad_Parameter;

        TFA_SET_BF(handle, DMEM, (uint16_t)Tfa98xx_DMEM_XMEM);

	error = -TFA_WRITE_REG(handle, MADD, start_offset);
	if (error != Tfa98xx_Error_Ok)
		return error;

	num_bytes = length; 
	while (num_bytes > 0) {
		burst_size = ROUND_DOWN(handles_local[handle].buffer_size, bytes_per_word);
		if (num_bytes < burst_size)
			burst_size = num_bytes;
		error =
		    tfa98xx_read_data(handle, FAM_TFA98XX_CF_MEM, burst_size, bytes + offset);
		if (error != Tfa98xx_Error_Ok)
			return error;

		num_bytes -= burst_size;
		offset += burst_size;
	}

	return Tfa98xx_Error_Ok;
}

#define MAX_WORDS (300)
enum Tfa98xx_Error tfa_dsp_msg(Tfa98xx_handle_t handle, int length, const char *buf)
{
	enum Tfa98xx_Error error;
	int tries, rpc_status = Tfa98xx_I2C_Req_Done;

	
	error = tfa_dsp_msg_write(handle, length, buf);
	if( error != Tfa98xx_Error_Ok)
		return error;

	
	for(tries=TFA98XX_WAITRESULT_NTRIES; tries>0;tries--) {
		error = tfa_dsp_msg_status(handle, &rpc_status);
                if (error == Tfa98xx_Error_Ok && rpc_status == Tfa98xx_I2C_Req_Done)
			break;
		if(rpc_status != Tfa98xx_I2C_Req_Busy && rpc_status != Tfa98xx_DSP_Not_Running)
			break;
	}

	
	

	if (rpc_status != Tfa98xx_I2C_Req_Done) {
		
		error = (enum Tfa98xx_Error) (rpc_status + Tfa98xx_Error_RpcBase);
                pr_debug("DSP msg status: %d (%s)\n", rpc_status, tfa98xx_get_i2c_status_id_string(rpc_status));
	} 

	return error;
}

enum Tfa98xx_Error tfa_dsp_msg_id(Tfa98xx_handle_t handle, int length, const char *buf, uint8_t cmdid[3])
{
	enum Tfa98xx_Error error;
	int tries, rpc_status = Tfa98xx_I2C_Req_Done;

	
	error = tfa_dsp_msg_write_id(handle, length, buf, cmdid);
	if( error != Tfa98xx_Error_Ok)
		return error;

	
	for(tries=TFA98XX_WAITRESULT_NTRIES; tries>0;tries--) {
		error = tfa_dsp_msg_status(handle, &rpc_status);
                if (error == Tfa98xx_Error_Ok && rpc_status == Tfa98xx_I2C_Req_Done)
			break;
	}

	
		

	if (rpc_status != Tfa98xx_I2C_Req_Done) {
		
		error = (enum Tfa98xx_Error) (rpc_status + Tfa98xx_Error_RpcBase);
                pr_debug("DSP msg status: %d (%s)\n", rpc_status, tfa98xx_get_i2c_status_id_string(rpc_status));
	} 

	return error;
}

TFA_INTERNAL enum Tfa98xx_Error
tfa98xx_check_rpc_status(Tfa98xx_handle_t handle, int *pRpcStatus)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned short cf_ctrl = 0x0002;
	
	unsigned short cf_mad = 0x0000;
	unsigned char mem[3];	

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;
	if (pRpcStatus == 0)
		return Tfa98xx_Error_Bad_Parameter;

	
#define OPTIMIZED_RPC
#ifdef OPTIMIZED_RPC 
	{
		unsigned char buffer[4];
		
		buffer[0] = (unsigned char)((cf_ctrl >> 8) & 0xFF);
		buffer[1] = (unsigned char)(cf_ctrl & 0xFF);
		buffer[2] = (unsigned char)((cf_mad >> 8) & 0xFF);
		buffer[3] = (unsigned char)(cf_mad & 0xFF);
		error =
		    tfa98xx_write_data(handle, FAM_TFA98XX_CF_CONTROLS,
				      sizeof(buffer), buffer);
	}
#else 

	if (error == Tfa98xx_Error_Ok) {
		error =
		    tfa98xx_write_register16(handle, FAM_TFA98XX_CF_CONTROLS,
					    cf_ctrl);
	}

	if (error == Tfa98xx_Error_Ok) {
		
		error = -TFA_WRITE_REG(handle, MADD, cf_mad);
	}
#endif 
	if (error == Tfa98xx_Error_Ok) {
		
		error =
				tfa98xx_read_data(handle, FAM_TFA98XX_CF_MEM, sizeof(mem), mem);
	}
	if (error == Tfa98xx_Error_Ok)
		*pRpcStatus = (mem[0] << 16) | (mem[1] << 8) | mem[2];
	return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_read_mem(Tfa98xx_handle_t handle,
		unsigned int start_offset, int num_words, int *pValues)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned char *bytes;
	int burst_size;		
	const int bytes_per_word = 3;
	int dmem;
	int num_bytes;
	int *p;

	bytes = (unsigned char *)kmalloc(num_words*bytes_per_word, GFP_KERNEL);
	if (bytes == NULL)
		return Tfa98xx_Error_Fail;

	
	if(((start_offset>>16) & 0xf) > 0 )
		dmem = (start_offset>>16) & 0xf;
	else
		dmem = Tfa98xx_DMEM_XMEM;

	TFA_SET_BF(handle, DMEM, (uint16_t)dmem);

	
	start_offset = start_offset & 0xffff;
	error = -TFA_WRITE_REG(handle, MADD, (unsigned short)start_offset);
	if (error != Tfa98xx_Error_Ok)
		goto tfa98xx_dsp_read_mem_exit;

	num_bytes = num_words * bytes_per_word;
	p = pValues;
	for (; num_bytes > 0;) {
		burst_size = ROUND_DOWN(handles_local[handle].buffer_size, bytes_per_word);
		if (num_bytes < burst_size)
			burst_size = num_bytes;

		_ASSERT(burst_size <= sizeof(bytes));
		error = tfa98xx_read_data( handle, FAM_TFA98XX_CF_MEM, burst_size, bytes);
		if (error != Tfa98xx_Error_Ok)
			goto tfa98xx_dsp_read_mem_exit;

		tfa98xx_convert_bytes2data(burst_size, bytes, p);

		num_bytes -= burst_size;
		p += burst_size / bytes_per_word;
	}

	tfa98xx_dsp_read_mem_exit:
	kfree(bytes);
	return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_write_mem_word(Tfa98xx_handle_t handle, unsigned short address, int value, int memtype)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned char bytes[3];

        TFA_SET_BF(handle, DMEM, (uint16_t)memtype);

	error = -TFA_WRITE_REG(handle, MADD, address);
	if (error != Tfa98xx_Error_Ok)
		return error;

	tfa98xx_convert_data2bytes(1, &value, bytes);
	error = tfa98xx_write_data(handle, FAM_TFA98XX_CF_MEM, 3, bytes);
	if (error != Tfa98xx_Error_Ok)
		return error;

	return Tfa98xx_Error_Ok;
}

enum Tfa98xx_Error tfa_cont_write_filterbank(int device, nxpTfaFilter_t *filter)
{
	unsigned char biquad_index;
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	for(biquad_index=0;biquad_index<10;biquad_index++) {
		if (filter[biquad_index].enabled ) {
			error = tfa_dsp_cmd_id_write(device, MODULE_BIQUADFILTERBANK,
					biquad_index+1, 
					sizeof(filter[biquad_index].biquad.bytes),
						filter[biquad_index].biquad.bytes);
		} else {
			error = Tfa98xx_DspBiquad_Disable(device, biquad_index+1);
		}
		if (error) return error;

	}
	return error;
}

enum Tfa98xx_Error
Tfa98xx_DspBiquad_Disable(Tfa98xx_handle_t handle, int biquad_index)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int coeff_buffer[BIQUAD_COEFF_SIZE];
	unsigned char bytes[3 + BIQUAD_COEFF_SIZE * 3];

	if (biquad_index > TFA98XX_BIQUAD_NUM)
		return Tfa98xx_Error_Bad_Parameter;
	if (biquad_index < 1)
		return Tfa98xx_Error_Bad_Parameter;

	

	
	coeff_buffer[0] = (int) - 8388608;	
	coeff_buffer[1] = 0;
	coeff_buffer[2] = 0;
	coeff_buffer[3] = 0;
	coeff_buffer[4] = 0;
	coeff_buffer[5] = 0;
	
	tfa98xx_convert_data2bytes(BIQUAD_COEFF_SIZE, coeff_buffer, &bytes[3]);

	bytes[0] = 0;
	bytes[1] = MODULE_BIQUADFILTERBANK+128;
	bytes[2] = (unsigned char)biquad_index;

	error = tfa_dsp_msg(handle, 3 + BIQUAD_COEFF_SIZE * 3, (char *)bytes);
	
	return error;
}

enum Tfa98xx_Error tfa_dsp_cmd_id_write(Tfa98xx_handle_t handle,
			   unsigned char module_id,
			   unsigned char param_id, int num_bytes,
                           const unsigned char data[])
{
	enum Tfa98xx_Error error;
	unsigned char *buffer;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	buffer = kmalloc(3 + num_bytes, GFP_KERNEL);
	if (buffer == NULL)
		return Tfa98xx_Error_Fail;

	buffer[0] = handles_local[handle].spkr_select;
	buffer[1] = module_id + 128;
	buffer[2] = param_id;

	memcpy(&buffer[3], data, num_bytes);

	error = tfa_dsp_msg(handle, 3 + num_bytes, (char *)buffer);

	kfree(buffer);

	return error;
}

enum Tfa98xx_Error tfa_dsp_cmd_id_write_read(Tfa98xx_handle_t handle,
			   unsigned char module_id,
			   unsigned char param_id, int num_bytes,
                           unsigned char data[])
{
	enum Tfa98xx_Error error;
	unsigned char buffer[3];

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	buffer[0] = handles_local[handle].spkr_select;
	buffer[1] = module_id + 128;
	buffer[2] = param_id;

	error = tfa_dsp_msg(handle, sizeof(unsigned char[3]), (char *)buffer);
	if ( error!= Tfa98xx_Error_Ok)
		return error;
        
	error = tfa_dsp_msg_read(handle, num_bytes, data);	

	return error;
}

enum Tfa98xx_Error tfa_dsp_cmd_id_coefs(Tfa98xx_handle_t handle,
			   unsigned char module_id,
			   unsigned char param_id, int num_bytes,
			   unsigned char data[])
{
	enum Tfa98xx_Error error;
	unsigned char buffer[6];

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	buffer[0] = handles_local[handle].spkr_select;
	buffer[1] = module_id + 128;
	buffer[2] = param_id;
        buffer[3] = 0;
        buffer[4] = 0;
        buffer[5] = 0;

	error = tfa_dsp_msg(handle, sizeof(unsigned char[6]), (char *)buffer);
	if (error != Tfa98xx_Error_Ok)
		return error;

	
	error = tfa_dsp_msg_read(handle, num_bytes, data);

	return error;
}

enum Tfa98xx_Error tfa_dsp_cmd_id_MBDrc_dynamics(Tfa98xx_handle_t handle,
			   unsigned char module_id,
			   unsigned char param_id, int index_subband,
			   int num_bytes, unsigned char data[])
{
	enum Tfa98xx_Error error;
	unsigned char buffer[6];

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	buffer[0] = handles_local[handle].spkr_select;
	buffer[1] = module_id + 128;
	buffer[2] = param_id;
        buffer[3] = 0;
        buffer[4] = 0;
        buffer[5] = (unsigned char)index_subband;

	error = tfa_dsp_msg(handle, sizeof(unsigned char[6]), (char *)buffer);
	if(error != Tfa98xx_Error_Ok)
		return error;

	
	error = tfa_dsp_msg_read(handle, num_bytes, data);

	return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_write_preset(Tfa98xx_handle_t handle, int length,
		       const unsigned char *p_preset_bytes)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	if (p_preset_bytes != 0) {
		error = tfa_dsp_cmd_id_write(handle, MODULE_SPEAKERBOOST,
					SB_PARAM_SET_PRESET, length,
					p_preset_bytes);
	} else {
		error = Tfa98xx_Error_Bad_Parameter;
	}
	return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_get_hw_feature_bits(Tfa98xx_handle_t handle, int *features)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	uint32_t value;
	uint16_t mtpbf;

	
	if (handles_local[handle].hw_feature_bits != -1) {
		*features = handles_local[handle].hw_feature_bits;
	} else {
		
		if (tfa98xx_dev_family(handle) == 1) {
			int status;
			tfa98xx_dsp_system_stable(handle, &status);
			if (!status) {
                                get_hw_features_from_cnt(handle, features);
                                
                                return (*features == -1) ? Tfa98xx_Error_Fail : Tfa98xx_Error_Ok; 
                        }
			mtpbf=0x850f;  
		} else
			mtpbf=0xf907;  
		value = tfa_read_reg(handle, mtpbf) & 0xffff;
		*features = handles_local[handle].hw_feature_bits = value;
	}

	return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_get_sw_feature_bits(Tfa98xx_handle_t handle, int features[2])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	unsigned char bytes[3 * 2];

	
	if (handles_local[handle].sw_feature_bits[0] != -1) {
		features[0] = handles_local[handle].sw_feature_bits[0];
		features[1] = handles_local[handle].sw_feature_bits[1];
	} else {
		
		if (tfa98xx_dev_family(handle) == 1) {
			int status;
			tfa98xx_dsp_system_stable(handle, &status);
			if (!status) {
                                get_sw_features_from_cnt(handle, features);
                                
                                return (features[0] == -1) ? Tfa98xx_Error_Fail : Tfa98xx_Error_Ok; 
                        }
		}
		error = tfa_dsp_cmd_id_write_read(handle, MODULE_FRAMEWORK,
				FW_PAR_ID_GET_FEATURE_INFO, sizeof(bytes), bytes);

		if (error != Tfa98xx_Error_Ok) {
			
			return error;
		}
		tfa98xx_convert_bytes2data(sizeof(bytes), bytes, features);
	}
	return error;
}

enum Tfa98xx_Error tfa98xx_dsp_get_state_info(Tfa98xx_handle_t handle, unsigned char bytes[], unsigned int *statesize)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
        int bSupportFramework = 0;
        unsigned int stateSize = 9;

	err = tfa98xx_dsp_support_framework(handle, &bSupportFramework);
	if (err == Tfa98xx_Error_Ok) {
		if (bSupportFramework) {
			err = tfa_dsp_cmd_id_write_read(handle, MODULE_FRAMEWORK,
				FW_PARAM_GET_STATE, 3 * stateSize, bytes);
		} else {
			
			stateSize = 8;
			err = tfa_dsp_cmd_id_write_read(handle, MODULE_SPEAKERBOOST,
				SB_PARAM_GET_STATE, 3 * stateSize, bytes);
		}
	}

	*statesize = stateSize;

	return err;
}

enum Tfa98xx_Error tfa98xx_dsp_support_drc(Tfa98xx_handle_t handle, int *pbSupportDrc)
{
    enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

    *pbSupportDrc = 0;

    if (!tfa98xx_handle_is_open(handle))
        return Tfa98xx_Error_NotOpen;
    if (handles_local[handle].supportDrc != supportNotSet) {
        *pbSupportDrc = (handles_local[handle].supportDrc == supportYes);
    } else {
        int featureBits[2];

        error = tfa98xx_dsp_get_sw_feature_bits(handle, featureBits);
        if (error == Tfa98xx_Error_Ok) {
            
            
            *pbSupportDrc = (featureBits[0] & FEATURE1_DRC) == 0;
        } else if (error == Tfa98xx_Error_RpcParamId) {
            
            *pbSupportDrc = 0;
            error = Tfa98xx_Error_Ok;
        }
        
        

        if (error == Tfa98xx_Error_Ok) {
        	handles_local[handle].supportDrc = *pbSupportDrc ? supportYes : supportNo;
        }
    }
    return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_support_framework(Tfa98xx_handle_t handle, int *pbSupportFramework)
{
    int featureBits[2] = { 0, 0 };
    enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

    _ASSERT(pbSupportFramework != 0);

    if (!tfa98xx_handle_is_open(handle))
        return Tfa98xx_Error_NotOpen;

    if (handles_local[handle].supportFramework != supportNotSet) {
	    if(handles_local[handle].supportFramework == supportNo)
		    *pbSupportFramework = 0;
	    else
		    *pbSupportFramework = 1;
    } else {
        error = tfa98xx_dsp_get_sw_feature_bits(handle, featureBits);
        if (error == Tfa98xx_Error_Ok) {
            *pbSupportFramework = 1;
            handles_local[handle].supportFramework = supportYes;
        } else {
            *pbSupportFramework = 0;
            handles_local[handle].supportFramework = supportNo;
            error = Tfa98xx_Error_Ok;
        }
    }

    
    return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_write_speaker_parameters(Tfa98xx_handle_t handle,
				  int length,
				  const unsigned char *p_speaker_bytes)
{
	enum Tfa98xx_Error error;
	if (p_speaker_bytes != 0) {
		
		error = tfa_dsp_cmd_id_write(
					handle,
					MODULE_SPEAKERBOOST,
					SB_PARAM_SET_LSMODEL, length,
					p_speaker_bytes);
	} else {
		error = Tfa98xx_Error_Bad_Parameter;
	}

#if (defined(TFA9887B) || defined(TFA98XX_FULL))
    {
        int bSupportDrc;

        if (error != Tfa98xx_Error_Ok)
            return error;

        error = tfa98xx_dsp_support_drc(handle, &bSupportDrc);
        if (error != Tfa98xx_Error_Ok)
            return error;

        if (bSupportDrc) {
        	uint8_t bytes[3] = {0, 0, 0};

        	error = tfa_dsp_cmd_id_write(handle,
        				     MODULE_SPEAKERBOOST,
        				     SB_PARAM_SET_AGCINS,
        				     sizeof(bytes),
        				     bytes);
        }
    }
#endif

    return error;
}

enum Tfa98xx_Error
tfa98xx_dsp_write_config(Tfa98xx_handle_t handle, int length,
               const unsigned char *p_config_bytes)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	error = tfa_dsp_cmd_id_write(handle,
				      MODULE_SPEAKERBOOST,
				      SB_PARAM_SET_CONFIG, length,
				      p_config_bytes);

#if (defined(TFA9887B) || defined(TFA98XX_FULL))
    {
        int bSupportDrc;

        if (error != Tfa98xx_Error_Ok)
            return error;

        error = tfa98xx_dsp_support_drc(handle, &bSupportDrc);
        if (error != Tfa98xx_Error_Ok)
            return error;

        if (bSupportDrc) {
        	uint8_t bytes[3] = {0, 0, 0};

        	error = tfa_dsp_cmd_id_write(handle,
        				     MODULE_SPEAKERBOOST,
        				     SB_PARAM_SET_AGCINS,
        				     sizeof(bytes),
        				     bytes);
        }
    }
#endif

    return error;
}

#if (defined(TFA9887B) || defined(TFA98XX_FULL))
enum Tfa98xx_Error tfa98xx_dsp_write_drc(Tfa98xx_handle_t handle,
                    int length, const unsigned char *p_drc_bytes)
{
    enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
    if (p_drc_bytes != 0) {
	error = tfa_dsp_cmd_id_write(handle,
				      MODULE_SPEAKERBOOST,
				      SB_PARAM_SET_DRC, length,
				      p_drc_bytes);

    } else {
        error = Tfa98xx_Error_Bad_Parameter;
    }
    return error;
}
#endif

enum Tfa98xx_Error tfa98xx_powerdown(Tfa98xx_handle_t handle, int powerdown)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

        TFA_SET_BF(handle, PWDN, (uint16_t)powerdown);

	return error;
}

enum Tfa98xx_Error
tfa98xx_select_mode(Tfa98xx_handle_t handle, enum Tfa98xx_Mode mode)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	if (!tfa98xx_handle_is_open(handle))
		return Tfa98xx_Error_NotOpen;

	if (error == Tfa98xx_Error_Ok) {
		switch (mode) {

		default:
			error = Tfa98xx_Error_Bad_Parameter;
		}
	}

	return error;
}

int tfa_set_bf(Tfa98xx_handle_t dev_idx, const uint16_t bf, const uint16_t value)
{
	enum Tfa98xx_Error err;
	uint16_t regvalue, msk, oldvalue;

	uint8_t len = bf & 0x0f;
	uint8_t pos = (bf >> 4) & 0x0f;
	uint8_t address = (bf >> 8) & 0xff;

	err = tfa98xx_read_register16(dev_idx, address, &regvalue);
	if (err) {
              pr_err("Error getting bf :%d \n", -err);
              return -err;
        }

        oldvalue = regvalue;
	msk = ((1<<(len+1))-1)<<pos;
	regvalue &= ~msk;
	regvalue |= value<<pos;

        
        if(oldvalue != regvalue) {
                err = tfa98xx_write_register16(dev_idx, address, regvalue);
	        if (err) {
                      pr_err("Error setting bf :%d \n", -err);
                      return -err;
                }
        }

	return 0;
}

int tfa_set_bf_volatile(Tfa98xx_handle_t dev_idx, const uint16_t bf, const uint16_t value)
{
	enum Tfa98xx_Error err;
	uint16_t regvalue, msk;

	uint8_t len = bf & 0x0f;
	uint8_t pos = (bf >> 4) & 0x0f;
	uint8_t address = (bf >> 8) & 0xff;

	err = tfa98xx_read_register16(dev_idx, address, &regvalue);
	if (err) {
              pr_err("Error getting bf :%d \n", -err);
              return -err;
        }

	msk = ((1<<(len+1))-1)<<pos;
	regvalue &= ~msk;
	regvalue |= value<<pos;

        err = tfa98xx_write_register16(dev_idx, address, regvalue);
	if (err) {
                pr_err("Error setting bf :%d \n", -err);
                return -err;
        }

	return 0;
}

int tfa_get_bf(Tfa98xx_handle_t dev_idx, const uint16_t bf)
{
	enum Tfa98xx_Error err;
	uint16_t regvalue, msk;
	uint16_t value;

	uint8_t len = bf & 0x0f;
	uint8_t pos = (bf >> 4) & 0x0f;
	uint8_t address = (bf >> 8) & 0xff;

	err = tfa98xx_read_register16(dev_idx, address, &regvalue);
        if (err) {
              pr_err("Error getting bf :%d \n", -err);
              return -err;
        }

	msk = ((1<<(len+1))-1)<<pos;
	regvalue &= msk;
	value = regvalue>>pos;

	return value;
}

int tfa_set_bf_value(const uint16_t bf, const uint16_t bf_value, uint16_t *p_reg_value)
{
	uint16_t regvalue, msk;

	uint8_t len = bf & 0x0f;
	uint8_t pos = (bf >> 4) & 0x0f;

	regvalue = *p_reg_value;

	msk = ((1<<(len+1))-1)<<pos;
	regvalue &= ~msk;
	regvalue |= bf_value<<pos;

	*p_reg_value = regvalue;

	return 0;
}

uint16_t tfa_get_bf_value(const uint16_t bf, const uint16_t reg_value)
{
	uint16_t msk, value;

	uint8_t len = bf & 0x0f;
	uint8_t pos = (bf >> 4) & 0x0f;

	msk = ((1<<(len+1))-1)<<pos;
	value = (reg_value & msk) >> pos;

	return value;
}


int tfa_write_reg(Tfa98xx_handle_t dev_idx, const uint16_t bf, const uint16_t reg_value)
{
	enum Tfa98xx_Error err;

	
	uint8_t address = (bf >> 8) & 0xff;

	err = tfa98xx_write_register16(dev_idx, address, reg_value);
	if (err)
		return -err;

	return 0;
}

int tfa_read_reg(Tfa98xx_handle_t dev_idx, const uint16_t bf)
{
	enum Tfa98xx_Error err;
	uint16_t regvalue;

	
	uint8_t address = (bf >> 8) & 0xff;

	err = tfa98xx_read_register16(dev_idx, address, &regvalue);
	if (err)
		return -err;

	return regvalue;
}

enum Tfa98xx_Error tfa_cf_powerup(Tfa98xx_handle_t handle) {
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int tries, status;

	
	TFA_SET_BF_VOLATILE(handle, PWDN, 0);

	
	if (tfa98xx_runtime_verbose)
		pr_info("Waiting for DSP system stable...\n");
	for ( tries=CFSTABLE_TRIES; tries > 0; tries-- ) {
		err = tfa98xx_dsp_system_stable(handle, &status);
		_ASSERT(err == Tfa98xx_Error_Ok);
		if ( status )
			break;
		else
			msleep_interruptible(10); 
	}
	if (tries==0) {
		pr_err("DSP subsystem start timed out\n");
		return Tfa98xx_Error_StateTimedOut;
	}

	return err;
}

static enum Tfa98xx_Error tfa98xx_aec_output(Tfa98xx_handle_t handle, int enable)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	if ((tfa98xx_get_device_dai(handle) & Tfa98xx_DAI_TDM) == Tfa98xx_DAI_TDM)
		return err;

	if (tfa98xx_dev_family(handle) == 1)
		err = -tfa_set_bf(handle, TFA1_BF_I2SDOE, (enable!=0));
	else {
		pr_err("I2SDOE on unsupported family\n");
		err = Tfa98xx_Error_Not_Supported;
	}

	return err;
}

enum Tfa98xx_Error show_current_state(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
        int manstate = -1;

	if (tfa98xx_dev_family(handle) == 2) {
		manstate = TFA_GET_BF(handle, MANSTATE);
		if (manstate < 0)
        		return -manstate;
	}

	pr_debug("Current HW manager state: ");

	switch(manstate) {
		case 0: pr_debug("power_down_state \n");
			break;
		case 1: pr_debug("wait_for_source_settings_state \n");
			break;
		case 2: pr_debug("connnect_pll_input_state \n");
			break;
		case 3: pr_debug("disconnect_pll_input_state \n");
			break;
		case 4: pr_debug("enable_pll_state \n");
			break;
		case 5: pr_debug("enable_cgu_state \n");
			break;
		case 6: pr_debug("init_cf_state \n");
			break;
		case 7: pr_debug("enable_amplifier_state \n");
			break;
		case 8: pr_debug("alarm_state \n");
			break;
		case 9: pr_debug("operating_state \n");
			break;
		case 10: pr_debug("mute_audio_state \n");
			break;
		case 11: pr_debug("disable_cgu_pll_state \n");
			break;
		default:
			pr_debug("Unable to find current state \n");
			break;
	}

	return err;
}

enum Tfa98xx_Error tfaRunSpeakerBoost(Tfa98xx_handle_t handle, int force, int profile)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
    int value;
    int istap_prof = 0;

	if ( force ) {
		err= tfaRunColdStartup(handle, profile);
		if ( err ) return err;
		
	}

	value = TFA_GET_BF(handle, ACS);

#ifdef __KERNEL__ 
	tfa98xx_trace_printk("%s %sstart\n",
	             tfaContDeviceName(handle),
	             value ? "cold" : "warm");
#endif
    istap_prof = tfaContIsTapProfile(handle, profile);

	if ((value == 1) && (!istap_prof)) {
		
		err = tfaRunSpeakerStartup(handle, force, profile);
		if ( err ) return err;

		
		
		tfa_set_swprof(handle, (unsigned short)profile);
		tfa_set_swvstep(handle, 0);

		
		if (TFA_GET_BF(handle, CFE) == 0) {
			return err;
		}

#ifdef __KERNEL__ 
		
		tfa98xx_apply_deferred_calibration(handle);
#endif
		
		err = tfaRunSpeakerCalibration(handle, profile);
	}
    else if (istap_prof) 
    {
        
		
		tfa_set_swprof(handle, (unsigned short)profile);
		tfa_set_swvstep(handle, 0);
    }

	return err;
}

enum Tfa98xx_Error tfaRunSpeakerStartup(Tfa98xx_handle_t handle, int force, int profile)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	pr_debug("coldstart%s :", force? " (forced)":"");

	if ( !force ) { 
		err = tfaRunStartup(handle, profile);
		PRINT_ASSERT(err);
		if ( err )
			return err;
		
		if (TFA_GET_BF(handle, CFE) == 0)
			return err;

		err = tfaRunStartDSP(handle);
		if ( err )
			return err;
	}
	
	
	
	
	
	
	
	
	
	
	
	
	err = tfaContWriteFiles(handle);
	if (err)
		return err;

	
	err = tfaContWriteFilesProf(handle, profile, 0); 
	if (err)
		return err;	

	return err;
}

enum Tfa98xx_Error tfaRunSpeakerCalibration(Tfa98xx_handle_t handle, int profile)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int calibrateDone, spkr_count = 0;
	
	profile=profile;

#ifdef __KERNEL__ 
	if((strstr(tfaContProfileName(handle, profile), ".cal") == NULL) && (tfa98xx_dev_family(handle) == 2))
    {
        TFA_SET_BF_VOLATILE(handle, SBSL, 1);
    }
    else if (tfa98xx_dev_family(handle) != 2)
#endif
		TFA_SET_BF_VOLATILE(handle, SBSL, 1);

	
	if (TFA_GET_BF(handle, NOCLK) && tfa98xx_dev_family(handle) == 2)
		return Tfa98xx_Error_NoClock;

	
	if (TFA_GET_BF(handle, MTPOTC) == 1) {
		tfa98xx_key2(handle, 0);
	}	

	
	err = tfaRunWaitCalibration(handle, &calibrateDone);
	
	if(err == Tfa98xx_Error_Ok) {
		err = tfa_dsp_get_calibration_impedance(handle);
		PRINT_ASSERT(err);
	}

	
	if(err != Tfa98xx_Error_Ok) {
		if ((tfa98xx_dev_family(handle) == 2 && TFA_GET_BF(handle, REFCKSEL) == 0)) {
			pr_err("Unable to calibrate the device with the internal clock! \n");
		}
	}

	if(err == Tfa98xx_Error_Ok) {
		err = tfa98xx_supported_speakers(handle, &spkr_count);

		if (spkr_count == 1) {
			pr_debug(" %d mOhms \n", handles_local[handle].mohm[0]);
		} else {
			pr_debug(" Prim:%d mOhms, Sec:%d mOhms\n",
						handles_local[handle].mohm[0], 
						handles_local[handle].mohm[1]);
		}
	}

	
	if (TFA_GET_BF(handle, MTPOTC) == 1) {
		tfa98xx_key2(handle, 1);
	}

	return err;
}

void tfa_verbose(int level)
{
	tfa98xx_trace_level = level;
	tfa98xx_runtime_verbose = level!=0; 
	tfa_cnt_verbose(level);
}

enum Tfa98xx_Error tfaRunColdboot(Tfa98xx_handle_t handle, int state)
{
#define CF_CONTROL 0x8100
	enum Tfa98xx_Error err=Tfa98xx_Error_Ok;
	int tries = 10;

	
	while ( state != TFA_GET_BF(handle, ACS)) {
		
                err = tfa98xx_dsp_write_mem_word(handle, CF_CONTROL, state, Tfa98xx_DMEM_IOMEM);
		PRINT_ASSERT(err);

		if (tries-- == 0) {
			pr_debug("coldboot (ACS) did not %s\n", state ? "set":"clear");
			return Tfa98xx_Error_Other;
		}
	}

	return err;
}



static enum Tfa98xx_Error tfa_run_load_patch(Tfa98xx_handle_t handle)
{
	return tfaContWritePatch(handle);
}

enum Tfa98xx_Error tfaRunStartDSP(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	err = tfa_run_load_patch(handle);
	if (err) { 
		return err;
	}

	
	err = tfa98xx_dsp_write_mem_word(handle, 512, 0, Tfa98xx_DMEM_XMEM);
	PRINT_ASSERT(err);

	
	if ( err == Tfa98xx_Error_Ok) {
		err = tfa98xx_dsp_reset(handle, 0); 
		PRINT_ASSERT(err);
	}

	
	err = tfa98xx_dsp_write_tables(handle, TFA_GET_BF(handle, AUDFS));
	PRINT_ASSERT(err);

	return err;
}

enum Tfa98xx_Error tfaRunStartup(Tfa98xx_handle_t handle, int profile)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	nxpTfaDeviceList_t *dev = tfaContDevice(handle);
	int tries, status, i, noinit=0;

	
	for(i=0;i<dev->length;i++) {
		if (dev->list[i].type == dscNoInit) {
			noinit=1;
			break;
		}
	}

	if(!noinit) {
		
		err = tfa98xx_init(handle);
		PRINT_ASSERT(err);
	} else {
		pr_debug("\nWarning: No init keyword found in the cnt file. Init is skipped! \n");
	}

	
	err = tfaContWriteRegsDev(handle); 
	PRINT_ASSERT(err);
	
	
	err = tfaContWriteRegsProf(handle, profile);
	PRINT_ASSERT(err);

	if(tfa98xx_dev_family(handle) == 2) {
		
		tfa_factory_trimmer(handle);
	}

	
	err = tfa98xx_powerdown(handle, 0);
	PRINT_ASSERT(err);

	if (tfa98xx_dev_family(handle) == 2) {
		TFA_SET_BF_VOLATILE(handle, MANSCONF, 1);
	}

	if (tfa98xx_runtime_verbose) {
		if (TFA_GET_BF(handle, NOCLK))
			pr_debug("Using internal clock\n");
		pr_debug("Waiting for DSP system stable...\n");
	}
	for ( tries=1; tries < CFSTABLE_TRIES; tries++ ) {
		err = tfa98xx_dsp_system_stable(handle, &status);
		_ASSERT(err == Tfa98xx_Error_Ok);
		if ( status )
			break;
		else
			msleep_interruptible(10); 
	}
	if (tries == CFSTABLE_TRIES) {
		if (tfa98xx_runtime_verbose) pr_debug("Timed out\n");
		return Tfa98xx_Error_StateTimedOut;
	}  else
		if (tfa98xx_runtime_verbose) pr_debug(" OK (tries=%d)\n", tries);

	if (tfa98xx_runtime_verbose && tfa98xx_dev_family(handle) == 2)
		err = show_current_state(handle);

	return err;
}

enum Tfa98xx_Error tfaRunColdStartup(Tfa98xx_handle_t handle, int profile)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	err = tfaRunStartup(handle, profile);
	PRINT_ASSERT(err);
	if (err)
		return err;

	
	err = tfaRunColdboot(handle, 1); 
	PRINT_ASSERT(err);
	if (err)
		return err;

	
	err = tfaRunStartDSP(handle);
	PRINT_ASSERT(err);

	return err;
}

enum Tfa98xx_Error tfaRunMute(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int status;
	int tries = 0;

	
	if(tfa98xx_dev_family(handle) == 1) {
		err = tfa98xx_set_mute(handle, Tfa98xx_Mute_Amplifier);

		if(err == Tfa98xx_Error_Ok) {
			
			do {
				status = TFA_GET_BF(handle, SWS);
				if (status != 0)
					msleep_interruptible(10); 
				else
					break;
				tries++;
			}  while (tries < AMPOFFWAIT_TRIES);


			if ( tfa98xx_runtime_verbose )
				pr_debug("-------------------- muted --------------------\n");

			
			if (tries == AMPOFFWAIT_TRIES)
				return Tfa98xx_Error_Other;
		}
	}

	return err;
}
enum Tfa98xx_Error tfaRunUnmute(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	
	err = tfa98xx_set_mute(handle, Tfa98xx_Mute_Off);

	if ( tfa98xx_runtime_verbose )
	    pr_debug("-------------------unmuted ------------------\n");

    return err;
}


enum Tfa98xx_Error tfaRunWaitCalibration(Tfa98xx_handle_t handle, int *calibrateDone)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int tries = 0, mtp_busy = 1, tries_mtp_busy = 0;

	*calibrateDone = 0;
	
	
	if (TFA_GET_BF(handle, MTPOTC)) {
		
		while (tries_mtp_busy < MTPBWAIT_TRIES)
		{
			mtp_busy = TFA_GET_BF(handle, MTPB);
			if (mtp_busy == 1)
				msleep_interruptible(10); 
			else
				break;
			tries_mtp_busy++;
		}

		if(tries_mtp_busy < MTPBWAIT_TRIES) {
			while ( (*calibrateDone == 0) && (tries < 25)) {	
				*calibrateDone = TFA_GET_BF(handle, MTPEX);
				if(*calibrateDone == 1)
					break;
				msleep_interruptible(50); 
				tries++;
			}

			if(tries >= 25) {
				tries = TFA98XX_API_WAITRESULT_NTRIES;
			}
		}
	}

	while ((*calibrateDone != 1) && (tries<TFA98XX_API_WAITRESULT_NTRIES)) {
		err = tfa98xx_dsp_read_mem(handle, TFA_FW_XMEM_CALIBRATION_DONE, 1, calibrateDone);
		tries++;
	}

	if(*calibrateDone != 1) {
		pr_err("Calibration failed! \n");
		err = Tfa98xx_Error_Bad_Parameter;
	} else if (tries==TFA98XX_API_WAITRESULT_NTRIES) {
		pr_debug("Calibration has timedout! \n");
		err = Tfa98xx_Error_StateTimedOut;
	} else if(tries_mtp_busy == 1000) {
		pr_err("Calibrate Failed: MTP_busy stays high! \n");
		err = Tfa98xx_Error_StateTimedOut;
	}

	
	if((err != Tfa98xx_Error_Ok) && ((handles_local[handle].rev & 0x0FFF) == 0xc88)) {
		individual_calibration_results(handle);
	}

#ifdef CONFIG_DEBUG_FS
	tfa98xx_deferred_calibration_status(handle, *calibrateDone);
#endif
	return err;
}

enum tfa_error tfa_start(int next_profile, int *vstep)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int dev, devcount = tfa98xx_cnt_max_device();
	int cal_profile = -1, istap_prof = 0, active_profile = -1;

	if ( devcount < 1 ) {
		pr_err("No or wrong container file loaded\n");
		return	tfa_error_bad_param;
	}

	for( dev=0; dev < devcount; dev++) {
		err = tfaContOpen(dev);
		if ( err != Tfa98xx_Error_Ok)
			goto error_exit;

		
		active_profile = tfa_get_swprof(dev);
		if (active_profile == 0xff)
			active_profile = -1;

		if (active_profile == -1 && next_profile < 1) {
			cal_profile = tfaContGetCalProfile(dev);
			if (cal_profile >= 0)
				next_profile = cal_profile;
		}
		istap_prof = tfaContIsTapProfile(dev, next_profile);

		
		if (tfa98xx_runtime_verbose) {
			pr_debug("active_profile:%s, next_profile:%s\n",
					tfaContProfileName(dev,active_profile ),
					tfaContProfileName(dev, next_profile));
			pr_debug("Starting device [%s]\n", tfaContDeviceName(dev));

			if(tfa98xx_dev_family(dev) == 2) {
				err = show_current_state(dev);
			}
		}

		
		err = tfa98xx_aec_output(dev, 1);
		if ( err != Tfa98xx_Error_Ok)
			goto error_exit;

		
		err = tfaRunSpeakerBoost(dev, 0, next_profile);
		if ( err != Tfa98xx_Error_Ok)
			goto error_exit;

		active_profile = tfa_get_swprof(dev);

		
		if (cal_profile >= 0) {
			next_profile = 0;
			pr_debug("Loading %s profile! \n", tfaContProfileName(dev, next_profile));
		}
	}

	for( dev=0; dev < devcount; dev++) {
		
		
		if (( next_profile != active_profile && active_profile != -1) 
            || (istap_prof == 1)) {
			err = tfaContWriteProfile(dev, next_profile, vstep[dev]);
			if (err!=Tfa98xx_Error_Ok)
				goto error_exit;
		} 

		if(strstr(tfaContProfileName(dev, next_profile), ".standby") != NULL) {
			err = tfa98xx_powerdown(dev, 1);
		} else if (TFA_GET_BF(dev, PWDN) != 0) {
			err = tfa98xx_powerdown(dev, 0);
		}

		if (err!=Tfa98xx_Error_Ok)
			goto error_exit;

		if (tfa98xx_runtime_verbose && tfa98xx_dev_family(dev) == 2)
			err = show_current_state(dev);

		
		err = tfa_set_filters(dev, next_profile);
		if (err!=Tfa98xx_Error_Ok)
			goto error_exit;

		
#ifdef __KERNEL__
		if(tfa98xx_dev_family(dev) == 2) {
			if((TFA_GET_BF(dev, RST) == 1) && (TFA_GET_BF(dev, SBSL) == 0) && (TFA_GET_BF(dev, MANSTATE) == 6)) {
				TFA_SET_BF_VOLATILE(dev, SBSL, 0);
				TFA_SET_BF(dev, RST, 0);	
			}
		}
#endif	

		if (vstep[dev] != tfaContGetCurrentVstep(dev) && vstep[dev] != -1) {
			err = tfaContWriteFilesVstep(dev, next_profile, vstep[dev]);
			if ( err != Tfa98xx_Error_Ok)
				goto error_exit;
		}

		tfa_set_swprof(dev, (unsigned short)next_profile);
		tfa_set_swvstep(dev, (unsigned short)tfaContGetCurrentVstep(dev));
	}

error_exit:
	if (tfa98xx_runtime_verbose && tfa98xx_dev_family(dev) == 2)
		show_current_state(dev);

	for( dev=0; dev < devcount; dev++) {
		tfaRunUnmute(dev);	
		tfaContClose(dev); 
	}

	return err;
}

enum tfa_error tfa_stop(void)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int dev, devcount = tfa98xx_cnt_max_device();

	if ( devcount == 0 ) {
		pr_err("No or wrong container file loaded\n");
		return	tfa_error_bad_param;
	}

	for( dev=0; dev < devcount; dev++) {
		err = tfaContOpen(dev);
		if ( err != Tfa98xx_Error_Ok)
			goto error_exit;
		if (tfa98xx_runtime_verbose)
			pr_debug("Stopping device [%s]\n", tfaContDeviceName(dev));
		
		tfaRunMute(dev);
		
		err = tfa98xx_powerdown(dev, 1 );
		if ( err != Tfa98xx_Error_Ok)
			goto error_exit;

		
		err = tfa98xx_aec_output(dev, 0);
		if ( err != Tfa98xx_Error_Ok)
			goto error_exit;
	}

error_exit:
	for( dev=0; dev < devcount; dev++)
		tfaContClose(dev); 
	return err;
}

int tfa98xx_reset(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	TFA_SET_BF_VOLATILE(handle, I2CR, 1);
	
	TFA_SET_BF_VOLATILE(handle, MANSCONF, 0);
	TFA_SET_BF_VOLATILE(handle, MANCOLD, 1);

	
	tfa98xx_powerdown(handle, 0 );
	
	err = tfa_cf_powerup(handle);
	PRINT_ASSERT(err);

	
	err = tfaRunColdboot(handle, 1); 
	PRINT_ASSERT(err);

	
	err = -TFA_SET_BF(handle, I2CR, 1);
	PRINT_ASSERT(err);

	return err;
}

enum tfa_error tfa_reset(void)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int dev, devcount = tfa98xx_cnt_max_device();

	for( dev=0; dev < devcount; dev++) {
		err = tfaContOpen(dev);
		if ( err != Tfa98xx_Error_Ok)
			break;
		if (tfa98xx_runtime_verbose)
			pr_debug("resetting device [%s]\n", tfaContDeviceName(dev));
		err = tfa98xx_reset(dev);
		if ( err != Tfa98xx_Error_Ok)
			break;
	}

	for( dev=0; dev < devcount; dev++) {
		tfaContClose(dev);
	}

	return err;
}

enum Tfa98xx_Error
tfa98xx_write_data(Tfa98xx_handle_t handle,
		  unsigned char subaddress, int num_bytes,
		  const unsigned char data[])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	
	const int bytes2write = num_bytes + 1;
	unsigned char *write_data;

	if (num_bytes > TFA2_MAX_PARAM_SIZE)
		return Tfa98xx_Error_Bad_Parameter;

	write_data = (unsigned char *)kmalloc(bytes2write, GFP_KERNEL);
	if (write_data == NULL)
		return Tfa98xx_Error_Fail;

	write_data[0] = subaddress;
	memcpy(&write_data[1], data, num_bytes);

	error = tfa98xx_write_raw(handle, bytes2write, write_data);

	kfree (write_data);
	return error;
}

enum Tfa98xx_Error
tfa_dsp_get_calibration_impedance(Tfa98xx_handle_t handle)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int spkr_count, nr_bytes, i;
	unsigned char bytes[6] = {0};
	int data[2];

	error = tfa98xx_supported_speakers(handle, &spkr_count);
	if (error == Tfa98xx_Error_Ok) {
		
		if (TFA_GET_BF(handle, MTPOTC) && ((handles_local[handle].rev & 0xff) == 0x88)) { 
			if (tfa98xx_runtime_verbose)
				pr_debug("Getting calibration values from MTP\n");
			for(i=0; i<spkr_count; i++) {
				handles_local[handle].mohm[i] = tfa_read_reg(handle, (uint16_t)TFA_MK_BF((0xF4+i),0,16));
			}
		} else {	
			
			if (tfa98xx_runtime_verbose)
				pr_debug("Getting calibration values from Speakerboost\n");
			nr_bytes = spkr_count * 3;
			error = tfa_dsp_cmd_id_write_read(handle,MODULE_SPEAKERBOOST,SB_PARAM_GET_RE0, nr_bytes, bytes);
			if (error == Tfa98xx_Error_Ok) {
				tfa98xx_convert_bytes2data(nr_bytes, bytes, data);
				for(i=0; i<spkr_count; i++) {
					handles_local[handle].mohm[i] = (data[i]*1000 )/TFA_FW_ReZ_SCALE;
				}
			} else {
				for(i=0; i<spkr_count; i++)
					handles_local[handle].mohm[i] = -1;
			}
		}
	}

	return error;
}

int tfa_get_swprof(Tfa98xx_handle_t handle) 
{
	
	if ( handles_local[handle].profile < 0)
		
		handles_local[handle].profile = TFA_GET_BF(handle, SWPROFIL)-1;

	return handles_local[handle].profile;
}

int tfa_set_swprof(Tfa98xx_handle_t handle, unsigned short new_value) {
	int mtpk, active_value = tfa_get_swprof(handle);

	handles_local[handle].profile=new_value;

	if ( handles_local[handle].tfa_family > 1 ) {
		TFA_SET_BF_VOLATILE(handle, SWPROFIL, new_value+1);
	} else {
		
		mtpk = TFA_GET_BF(handle, MTPK); 
		TFA_SET_BF_VOLATILE(handle, MTPK, 0x5a);
		TFA_SET_BF_VOLATILE(handle, SWPROFIL, new_value+1); 
		TFA_SET_BF_VOLATILE(handle, MTPK, (uint16_t)mtpk); 
	}

	return active_value;
}

int tfa_get_swvstep(Tfa98xx_handle_t handle){
	int value;

	if ( handles_local[handle].vstep[0]>0)
		return handles_local[handle].vstep[0]-1;

	value = TFA_GET_BF(handle, SWVSTEP); 

	handles_local[handle].vstep[0] = value;
	handles_local[handle].vstep[1] = value;
	return value-1; 

}
int tfa_set_swvstep(Tfa98xx_handle_t handle, unsigned short new_value) {
	int mtpk, active_value = tfa_get_swvstep(handle);

	handles_local[handle].vstep[0]=new_value;
	handles_local[handle].vstep[1]=new_value;

	if ( handles_local[handle].tfa_family > 1 ) {
		TFA_SET_BF_VOLATILE(handle, SWVSTEP, new_value+1);
	} else {
		
		mtpk = TFA_GET_BF(handle, MTPK); 
		TFA_SET_BF_VOLATILE(handle, MTPK, 0x5a);
		TFA_SET_BF_VOLATILE(handle, SWVSTEP, new_value+1); 
		TFA_SET_BF_VOLATILE(handle, MTPK, (uint16_t)mtpk); 
	}

	return active_value;
}

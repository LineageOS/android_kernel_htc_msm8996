/*
 *Copyright 2014 NXP Semiconductors
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

#include "tfa_service.h"
#include "tfa_container.h"
#include "config.h"
#include "tfa.h"
#include "tfa_dsp_fw.h"
#include "tfa98xx_tfafieldnames.h"

static int tfa98xx_cnt_verbose;

static nxpTfaContainer_t *g_cont=NULL; 
static int g_devs=-1; 
static nxpTfaDeviceList_t *g_dev[TFACONT_MAXDEVS];
static int g_profs[TFACONT_MAXDEVS];
static int g_liveds[TFACONT_MAXDEVS];
static nxpTfaProfileList_t  *g_prof[TFACONT_MAXDEVS][TFACONT_MAXPROFS];
static nxpTfaLiveDataList_t  *g_lived[TFACONT_MAXDEVS][TFACONT_MAXPROFS];
static int nxp_tfa_vstep[TFACONT_MAXDEVS];
static char errorname[] = "!ERROR!";
static char nonename[] = "NONE";

static void cont_get_devs(nxpTfaContainer_t *cont);

static int float_to_int(uint32_t x)
{
	unsigned e = (0x7F + 31) - ((*(unsigned *) &x & 0x7F800000) >> 23);
	unsigned m = 0x80000000 | (*(unsigned *) &x << 8);
	return -(int)((m >> e) & -(e < 32));
}

enum tfa_error tfa_load_cnt(void *cnt, int length) {
	nxpTfaContainer_t  *cntbuf = (nxpTfaContainer_t  *)cnt;

	g_cont = NULL;

	if( length>TFA_MAX_CNT_LENGTH )
		return tfa_error_container;

	if ( (HDR(cntbuf->id[0],cntbuf->id[1])) != paramsHdr ) {
		pr_err("wrong header type: 0x%02x 0x%02x\n", cntbuf->id[0],cntbuf->id[1]); 
		return tfa_error_container;
	}

	
	if ( tfaContCrcCheckContainer(cntbuf)) {
		pr_err("CRC error\n");
		return tfa_error_container;
	}

	
	if ( (cntbuf->subversion[1] == NXPTFA_PM_SUBVERSION) &&
		 (cntbuf->subversion[0] == '0') ) {
		g_cont = cntbuf;
		cont_get_devs(g_cont);
	} else {
		pr_err("container sub-version not supported: %c%c\n",
				cntbuf->subversion[0], cntbuf->subversion[1]);
		return tfa_error_container;
	}

	return tfa_error_ok;
}

void tfa_deinit(void)
{
	g_cont = NULL;
	g_devs = -1;
}

void tfa_cnt_verbose(int level) {
	tfa98xx_cnt_verbose = level;
}

void tfaContSetCurrentVstep(int channel, int vstep_idx) {
	if( channel<TFACONT_MAXDEVS)
		nxp_tfa_vstep[channel] = vstep_idx+1;
	else
		pr_err("channel nr %d>%d\n", channel, TFACONT_MAXDEVS-1);
}

int tfaContGetCurrentVstep(int channel) {
	if( channel<TFACONT_MAXDEVS)
		return nxp_tfa_vstep[channel]-1;

	pr_err("channel nr %d>%d\n", channel, TFACONT_MAXDEVS-1);
	return -1;
}

nxpTfaContainer_t * tfa98xx_get_cnt(void) {
	return g_cont;
}

void tfaContShowHeader(nxpTfaHeader_t *hdr) {
	char _id[2];

	pr_debug("File header\n");

	_id[1] = hdr->id >> 8;
	_id[0] = hdr->id & 0xff;
	pr_debug("\tid:%.2s version:%.2s subversion:%.2s\n", _id,
		   hdr->version, hdr->subversion);
	pr_debug("\tsize:%d CRC:0x%08x \n", hdr->size, hdr->CRC);
	pr_debug( "\tcustomer:%.8s application:%.8s type:%.8s\n", hdr->customer,
			   hdr->application, hdr->type);
}

nxpTfaDeviceList_t *tfaContGetDevList(nxpTfaContainer_t * cont, int dev_idx)
{
	uint8_t *base = (uint8_t *) cont;

	if ( (dev_idx < 0) || (dev_idx >= cont->ndev))
		return NULL;

	if (cont->index[dev_idx].type != dscDevice)
		return NULL;

	base += cont->index[dev_idx].offset;
	return (nxpTfaDeviceList_t *) base;
}

nxpTfaProfileList_t *tfaContGetDevProfList(nxpTfaContainer_t * cont, int devIdx,
					   int profIdx)
{
	nxpTfaDeviceList_t *dev;
	int idx, hit;
	uint8_t *base = (uint8_t *) cont;

	dev = tfaContGetDevList(cont, devIdx);
	if (dev) {
		for (idx = 0, hit = 0; idx < dev->length; idx++) {
			if (dev->list[idx].type == dscProfile) {
				if (profIdx == hit++)
					return (nxpTfaProfileList_t *) (dev->
									list
									[idx].
									offset +
									base);
			}
		}
	}

	return NULL;
}

nxpTfaLiveDataList_t *tfaContGetDevLiveDataList(nxpTfaContainer_t * cont, int devIdx,
					   int lifeDataIdx)
{
	nxpTfaDeviceList_t *dev;
	int idx, hit;
	uint8_t *base = (uint8_t *) cont;

	dev = tfaContGetDevList(cont, devIdx);
	if (dev) {
		for (idx = 0, hit = 0; idx < dev->length; idx++) {
			if (dev->list[idx].type == dscLiveData) {
				if (lifeDataIdx == hit++)
                                        return (nxpTfaLiveDataList_t *) 
                                        (dev->list[idx].offset + base);
			}
		}
	}

	return NULL;
}

int tfacont_get_max_vstep(int dev_idx, int prof_idx) {
	nxpTfaVolumeStep2File_t *vp;
	struct nxpTfaVolumeStepMax2File *vp3;
	int vstep_count = 0;
	vp = (nxpTfaVolumeStep2File_t *) tfacont_getfiledata(dev_idx, prof_idx, volstepHdr);
	if (vp == NULL)
		return 0;
	
	if (tfa98xx_dev_family(dev_idx) == 2) {
		
		vp3 = (struct nxpTfaVolumeStepMax2File *)
		tfacont_getfiledata(dev_idx, prof_idx, volstepHdr); 
		if ( vp3 ) {
			vstep_count = vp3->NrOfVsteps;
		}
	} else {
		
		if ( vp ) {
			vstep_count = vp->vsteps;
		}
	}
	return vstep_count;
}

nxpTfaFileDsc_t *tfacont_getfiledata(int dev_idx, int prof_idx, enum nxpTfaHeaderType type)
{
	nxpTfaDeviceList_t *dev;
	nxpTfaProfileList_t *prof;
	nxpTfaFileDsc_t *file;
	nxpTfaHeader_t *hdr;
	unsigned int i;

	if( g_cont==0 )
		return NULL;

	dev = tfaContGetDevList(g_cont, dev_idx);

	if( dev==0 )
		return NULL;

	
	for(i=0;i<dev->length;i++) {
		if ( dev->list[i].type == dscFile ) {
			file = (nxpTfaFileDsc_t *)(dev->list[i].offset+(uint8_t *)g_cont);
			hdr = (nxpTfaHeader_t *)file->data;
			
			if ( hdr->id == type) {
				
				return (nxpTfaFileDsc_t *)&file->data;
			}
		}
	}

	prof=tfaContGetDevProfList(g_cont, dev_idx, prof_idx);
	for(i=0;i<prof->length;i++) {
		if (prof->list[i].type == dscFile) {
			file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)g_cont);
			hdr= (nxpTfaHeader_t *)file->data;
			
			if ( hdr->id == type) {
				
				return (nxpTfaFileDsc_t *)&file->data;
			}
		}
	}

	if ( tfa98xx_cnt_verbose )
		pr_debug("%s: no file found of type %d\n", __FUNCTION__, type);

	return NULL;
}

static void cont_get_devs(nxpTfaContainer_t *cont) {
	nxpTfaProfileList_t *prof;
        nxpTfaLiveDataList_t *liveD;
	int i,j;
	int count;

	
	for(i=0 ; i < cont->ndev ; i++) {
		g_dev[i] = tfaContGetDevList(cont, i); 
	}

	g_devs=cont->ndev;
	
	for (i = 0; i < g_devs; i++) {
		j=0;
		count=0;
		while ((prof = tfaContGetDevProfList(cont, i, j)) != NULL) {
			count++;
			g_prof[i][j++] = prof;
		}
		g_profs[i] = count;    
	}

        g_devs=cont->ndev;
	
	for (i = 0; i < g_devs; i++) {
		j=0;
		count=0;
		while ((liveD = tfaContGetDevLiveDataList(cont, i, j)) != NULL) {
			count++;
			g_lived[i][j++] = liveD;
		}
		g_liveds[i] = count;    
	}
}

static char nostring[]="Undefined string";

#define MODULE_BIQUADFILTERBANK 2
#define BIQUAD_COEFF_SIZE       6
static enum Tfa98xx_Error tfaContWriteVstep(int dev_idx,  nxpTfaVolumeStep2File_t *vp, int vstep) 
{
	enum Tfa98xx_Error err;
	float voldB = 0.0;
	unsigned short vol;

	if (vstep < vp->vsteps) {
		voldB = vp->vstep[vstep].attenuation;
		
		vol = (unsigned short)(-2 * float_to_int(*((uint32_t *)&voldB)));
		if (vol > 255)	
			vol = 255;

		err = tfa98xx_set_volume_level(dev_idx, vol);
		if (err != Tfa98xx_Error_Ok)
			return err;

		err = tfa98xx_dsp_write_preset( dev_idx, sizeof(vp->vstep[0].preset), vp->vstep[vstep].preset);
		if (err != Tfa98xx_Error_Ok)
			return err;
		err = tfa_cont_write_filterbank(dev_idx, vp->vstep[vstep].filter);

	} else {
		pr_err("Incorrect volume given. The value vstep[%d] >= %d\n", nxp_tfa_vstep[dev_idx] , vp->vsteps);
		err = Tfa98xx_Error_Bad_Parameter;
	}

	if ( tfa98xx_cnt_verbose ) pr_debug("vstep[%d][%d]\n", dev_idx, vstep);

	return err;
}

static enum Tfa98xx_Error tfaContWriteVstepMax2(int dev_idx, nxpTfaVolumeStepMax2File_t *vp, int vstep_idx, int vstep_msg_idx)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	struct nxpTfaVolumeStepRegisterInfo *regInfo = {0};
	struct nxpTfaVolumeStepMessageInfo *msgInfo = {0};
	nxpTfaBitfield_t bitF;
	int msgLength=0, i, j, size=0, nrMessages, modified=0;
	uint8_t cmdid_changed[3];

	if(vstep_idx >= vp->NrOfVsteps) {
		pr_debug("Volumestep %d is not available \n", vstep_idx);
		return Tfa98xx_Error_Bad_Parameter;
	}

	for(i=0; i<=vstep_idx; i++) {
		regInfo = (struct nxpTfaVolumeStepRegisterInfo*)(vp->vstepsBin + size);
		msgInfo = (struct nxpTfaVolumeStepMessageInfo*)(vp->vstepsBin+
				(regInfo->NrOfRegisters * sizeof(uint32_t)+sizeof(regInfo->NrOfRegisters)+size));

		nrMessages = msgInfo->NrOfMessages;
		for(j=0; j<nrMessages; j++) {
			
			msgInfo = (struct nxpTfaVolumeStepMessageInfo*)(vp->vstepsBin+
					(regInfo->NrOfRegisters * sizeof(uint32_t)+sizeof(regInfo->NrOfRegisters)+size));
			
			msgLength = ( (msgInfo->MessageLength.b[0] << 16) + (msgInfo->MessageLength.b[1] << 8) + msgInfo->MessageLength.b[2]);
			if (i == vstep_idx) {
				
				if ((vstep_msg_idx >= TFA_MAX_VSTEP_MSG_MARKER) || (vstep_msg_idx == j)) {
					if(TFA_GET_BF(dev_idx, SBSL) == 0) {
						if(msgInfo->MessageType == 0) { 
							
							if(msgInfo->CmdId[2] != SB_PARAM_SET_ALGO_PARAMS) {
								cmdid_changed[0] = msgInfo->CmdId[0];
								cmdid_changed[1] = msgInfo->CmdId[1];
								cmdid_changed[2] = SB_PARAM_SET_ALGO_PARAMS;
								modified = 1;
							}
						} else if(msgInfo->MessageType == 2) { 
							
							if(msgInfo->CmdId[2] != SB_PARAM_SET_MBDRC) {
								cmdid_changed[0] = msgInfo->CmdId[0];
								cmdid_changed[1] = msgInfo->CmdId[1];
								cmdid_changed[2] = SB_PARAM_SET_MBDRC;
								modified = 1;
							}
						}
					}
					
					if(msgInfo->MessageType != 3) { 
						if(modified == 1) {
							if (tfa98xx_cnt_verbose) {
								if(cmdid_changed[2] == SB_PARAM_SET_ALGO_PARAMS)
									pr_debug("P-ID for SetAlgoParams modified!: ");
								else
									pr_debug("P-ID for SetMBDrc modified!: ");

								pr_debug("Command-ID used: 0x%02x%02x%02x \n", 
									cmdid_changed[0], cmdid_changed[1], cmdid_changed[2]);
							}
							
							err = tfa_dsp_msg_id(dev_idx, (msgLength-1) * 3, (const char *)msgInfo->ParameterData, cmdid_changed);
							if (err != Tfa98xx_Error_Ok)
								return err;
						} else {
							
							err = tfa_dsp_msg(dev_idx, msgLength * 3,(const char *)msgInfo->CmdId);
							if (err != Tfa98xx_Error_Ok)
								return err;
						}
					
						
						modified = 0;
					}
				}
			}

			if(msgInfo->MessageType == 3) {
				
				size += sizeof(msgInfo->MessageType) + sizeof(msgInfo->MessageLength) + msgLength;
			} else {
				
				size += sizeof(msgInfo->MessageType) + sizeof(msgInfo->MessageLength) + sizeof(msgInfo->CmdId) + ((msgLength-1) * 3);
			}
		}
		size += sizeof(regInfo->NrOfRegisters) + (regInfo->NrOfRegisters * sizeof(uint32_t)) + sizeof(msgInfo->NrOfMessages);
	}

	if (regInfo->NrOfRegisters == 0) {
		pr_debug("No registers in selected vstep (%d)!\n", vstep_idx);
		return Tfa98xx_Error_Bad_Parameter;
	}

	for(i=0; i<regInfo->NrOfRegisters*2; i++) {
		
		bitF.field = (uint16_t)(regInfo->registerInfo[i]>>8) | (regInfo->registerInfo[i]<<8);
		i++;
		bitF.value = (uint16_t)regInfo->registerInfo[i]>>8;
		err = tfaRunWriteBitfield(dev_idx , bitF);
		if (err != Tfa98xx_Error_Ok)
			return err;
	}

	
	tfa_set_swvstep(dev_idx, (unsigned short)vstep_idx);

	return err;
}


enum Tfa98xx_Error tfaContWriteDrcFile(int dev_idx, int size, uint8_t data[])
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	uint8_t cmdid_changed[3], modified = 0;

	if(TFA_GET_BF(dev_idx, SBSL) == 0) {
		
		if(data[2] != SB_PARAM_SET_MBDRC) {
			cmdid_changed[0] = data[0];
			cmdid_changed[1] = data[1];
			cmdid_changed[2] = SB_PARAM_SET_MBDRC;
			modified = 1;

			if (tfa98xx_cnt_verbose) {
				pr_debug("P-ID for SetMBDrc modified!: ");
				pr_debug("Command-ID used: 0x%02x%02x%02x \n", 
				cmdid_changed[0], cmdid_changed[1], cmdid_changed[2]);
			}
		}
	}

	if(modified == 1) {
		
		err = tfa_dsp_msg_id(dev_idx, size-3, (const char *)data, cmdid_changed);
	} else {
		
		err = tfa_dsp_msg(dev_idx, size, (const char *)data);
	}

	return err;
}


enum Tfa98xx_Error tfaContWriteFile(int dev_idx,  nxpTfaFileDsc_t *file, int vstep_idx, int vstep_msg_idx)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	nxpTfaHeader_t *hdr = (nxpTfaHeader_t *)file->data;
	nxpTfaHeaderType_t type;
	int size;	

	if ( tfa98xx_cnt_verbose ) {
		tfaContShowHeader(hdr);
	}

	type = (nxpTfaHeaderType_t) hdr->id;

	switch (type) {
	case msgHdr: 
		size = hdr->size - sizeof(nxpTfaMsgFile_t);
		err = tfa_dsp_msg(dev_idx, size, (const char *)((nxpTfaMsgFile_t *)hdr)->data);
		break;
	case volstepHdr:
		if (tfa98xx_dev_family(dev_idx) == 2) {
			err = tfaContWriteVstepMax2(dev_idx, (nxpTfaVolumeStepMax2File_t *)hdr, vstep_idx, vstep_msg_idx);
		} else {
			err = tfaContWriteVstep(dev_idx, (nxpTfaVolumeStep2File_t *)hdr, vstep_idx);
		}

		
		if(err == Tfa98xx_Error_Ok) {
			tfaContSetCurrentVstep(dev_idx, vstep_idx);
		}

		break;
	case speakerHdr:
		if (tfa98xx_dev_family(dev_idx) == 2) {
			
			size = hdr->size - sizeof(struct nxpTfaSpkHeader) - sizeof(struct nxpTfaFWVer);

			err = tfa_dsp_msg(dev_idx, size,
					(const char *)(((nxpTfaSpeakerFile_t *)hdr)->data + (sizeof(struct nxpTfaFWVer))));
		} else {
			size = hdr->size - sizeof(nxpTfaSpeakerFile_t);
			err = tfa98xx_dsp_write_speaker_parameters( dev_idx, size,
					(const unsigned char *)((nxpTfaSpeakerFile_t *)hdr)->data);
		}
		break;
	case presetHdr:
		size = hdr->size - sizeof(nxpTfaPreset_t);
		err = tfa98xx_dsp_write_preset( dev_idx, size, (const unsigned char *)((nxpTfaPreset_t *)hdr)->data);
		break;
	case equalizerHdr:
		err = tfa_cont_write_filterbank(dev_idx, ((nxpTfaEqualizerFile_t *)hdr)->filter);
		break;
	case patchHdr:
		size = hdr->size - sizeof(nxpTfaPatch_t ); 
		err = tfa_dsp_patch(dev_idx,  size, (const unsigned char *) ((nxpTfaPatch_t *)hdr)->data);
		break;
	case configHdr:
		size = hdr->size - sizeof(nxpTfaConfig_t);
		err = tfa98xx_dsp_write_config(dev_idx, size, (const unsigned char *)((nxpTfaConfig_t *)hdr)->data);
		break;
	case drcHdr:
		if(hdr->version[0] == NXPTFA_DR3_VERSION) {
			
			size = hdr->size - sizeof(nxpTfaDrc2_t);
			err = tfaContWriteDrcFile(dev_idx, size, ((nxpTfaDrc2_t *)hdr)->data);
		} else {
			
			size = 381; 

			
			err = tfa98xx_dsp_write_drc( dev_idx, size, ((const unsigned char *)((nxpTfaDrc_t *)hdr)->data+381));
		}
		break;
	case infoHdr:
		
		break;
	default:
		pr_err("Header is of unknown type: 0x%x\n", type);
		return Tfa98xx_Error_Bad_Parameter;
	}

	return err;
}

nxpTfaDescPtr_t *tfa_cnt_get_dsc(nxpTfaContainer_t *cnt, nxpTfaDescriptorType_t type, int dev_idx)
{
	nxpTfaDeviceList_t *dev = tfaContDevice (dev_idx);
	nxpTfaDescPtr_t *this;
	int i;

	if ( !dev ) {
		return NULL;
	}
	
	for(i=0;i<dev->length;i++) {
		if ( dev->list[i].type == (uint32_t)type ) {
			this = (nxpTfaDescPtr_t *)(dev->list[i].offset+(uint8_t *)cnt);
			return this;
		}

	}

	return NULL;
}

int tfa_cnt_get_devid(nxpTfaContainer_t *cnt, int dev_idx) {
	nxpTfaPatch_t *patchfile;
	nxpTfaDescPtr_t *patchdsc;
	uint8_t *patchheader;
	unsigned short devid, checkaddress;
	int checkvalue;

	patchdsc = tfa_cnt_get_dsc(cnt, dscPatch, dev_idx);
	patchdsc += 2; 
	patchfile = (nxpTfaPatch_t *)patchdsc;

	if (patchfile==NULL)
		return 0;

	patchheader = patchfile->data;

	checkaddress = (patchheader[1] << 8) + patchheader[2];
	checkvalue = 
		(patchheader[3] << 16) + (patchheader[4] << 8) + patchheader[5];

	devid = patchheader[0];

	if(checkaddress == 0xFFFF && checkvalue != 0xFFFFFF && checkvalue != 0) {
		devid = patchheader[5]<<8 | patchheader[0]; 
	}

	return devid;
}

enum Tfa98xx_Error tfaContGetSlave(int dev_idx, uint8_t *slave_addr) {
	nxpTfaDeviceList_t *dev = tfaContDevice (dev_idx);

	if (dev==0 ) {
		return Tfa98xx_Error_Bad_Parameter;
	}

	*slave_addr = dev->dev;
	return Tfa98xx_Error_Ok;
}

enum Tfa98xx_Error tfaRunWriteBitfield(Tfa98xx_handle_t dev_idx,  nxpTfaBitfield_t bf) {
	enum Tfa98xx_Error error;
        uint16_t value;
	union {
		uint16_t field;
		nxpTfaBfEnum_t Enum;
	} bfUni;
        
	value=bf.value;
	bfUni.field = bf.field;
#ifdef TFA_DEBUG
	if ( tfa98xx_cnt_verbose )
		pr_debug("bitfield: %s=%d (0x%x[%d..%d]=0x%x)\n", tfaContBfName(bfUni.field, tfa98xx_dev_revision(dev_idx)), value,
			bfUni.Enum.address, bfUni.Enum.pos, bfUni.Enum.pos+bfUni.Enum.len, value);
#endif
        error = tfa_set_bf(dev_idx, bfUni.field, value);

	return error;
}

enum Tfa98xx_Error tfaRunReadBitfield(Tfa98xx_handle_t dev_idx,  nxpTfaBitfield_t *bf) {
	enum Tfa98xx_Error error;
	union {
		uint16_t field;
		nxpTfaBfEnum_t Enum;
	} bfUni;
	uint16_t regvalue, msk;

	bfUni.field = bf->field;

	error = tfa98xx_read_register16(dev_idx, (unsigned char)(bfUni.Enum.address), &regvalue);
	if (error) return error;

	msk = ((1<<(bfUni.Enum.len+1))-1)<<bfUni.Enum.pos;

	regvalue &= msk;
	bf->value = regvalue>>bfUni.Enum.pos;

	return error;
}

enum Tfa98xx_Error tfaRunWriteDspMem(Tfa98xx_handle_t dev, nxpTfaDspMem_t *cfmem)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int i;

	for(i=0;i<cfmem->size;i++) {
		if ( tfa98xx_cnt_verbose )
			pr_debug("dsp mem (%d): 0x%02x=0x%04x\n", cfmem->type, cfmem->address, cfmem->words[i]);

		error = tfa98xx_dsp_write_mem_word(dev, cfmem->address++, cfmem->words[i], cfmem->type);
		if (error) return error;
	}

	return error;
}

enum Tfa98xx_Error tfaRunWriteFilter(Tfa98xx_handle_t dev, nxpTfaContBiquad_t *bq) {
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	enum Tfa98xx_DMEM dmem;
	uint16_t address;
	uint8_t data[3*3+sizeof(bq->aa.bytes)];
	int i, channel=0, runs=1;
	int8_t saved_index=bq->aa.index; 

	
	if (bq->aa.index > 100) {
		bq->aa.index -= 100;
		channel = 2;
	} else if (bq->aa.index > 50) {
		bq->aa.index -= 50;
		channel = 1;
	} else if(tfa98xx_dev_family(dev) == 2) {
			runs=2;
	}

	if ( tfa98xx_cnt_verbose ) {
		if(channel == 2)
			pr_debug("filter[%d,S]", bq->aa.index);
		else if(channel == 1)
			pr_debug("filter[%d,P]", bq->aa.index);
		else
			pr_debug("filter[%d]", bq->aa.index);
	}

	for(i=0; i<runs; i++) {
		if(runs==2) 
			channel++;

		
		dmem = tfa98xx_filter_mem(dev, bq->aa.index, &address, channel);
		if(dmem<0)
			return Tfa98xx_Error_Bad_Parameter;

		memset(data, 0, 3*3);
		data[2] = dmem; 
		data[4] = address >> 8; 
		data[5] = address & 0xff;
		data[8] = sizeof(bq->aa.bytes)/3; 
		memcpy( &data[9], bq->aa.bytes, sizeof(bq->aa.bytes)); 

		if(tfa98xx_dev_family(dev) == 2) {
			error = tfa_dsp_cmd_id_write(dev, MODULE_FRAMEWORK, FW_PAR_ID_SET_MEMORY, sizeof(data), data);
		} else {
			error = tfa_dsp_cmd_id_write(dev, MODULE_FRAMEWORK, 4  , sizeof(data), data);
		}
	}

#ifdef TFA_DEBUG
	if ( tfa98xx_cnt_verbose ) {			
		if (bq->aa.index==13) {
			pr_debug("=%d,%.0f,%.2f \n", 
				bq->in.type, bq->in.cutOffFreq, bq->in.leakage);
		} else if(bq->aa.index >= 10 && bq->aa.index <= 12) {
			pr_debug("=%d,%.0f,%.1f,%.1f \n", bq->aa.type, 
				bq->aa.cutOffFreq, bq->aa.rippleDb, bq->aa.rolloff);
		} else {
			pr_debug("= unsupported filter index \n");
		}
	}
#endif

	bq->aa.index = saved_index;

	return error;
}

enum Tfa98xx_Error tfaRunWriteRegister(Tfa98xx_handle_t handle, nxpTfaRegpatch_t *reg)
{
	enum Tfa98xx_Error error;
	uint16_t value,newvalue;

	if ( tfa98xx_cnt_verbose )
		pr_debug("register: 0x%02x=0x%04x (msk=0x%04x)\n", reg->address, reg->value, reg->mask);

	error = tfa98xx_read_register16(handle, reg->address, &value);
	if (error) return error;

	value &= ~reg->mask;
	newvalue = reg->value & reg->mask;

	value |= newvalue;
	error = tfa98xx_write_register16(handle,  reg->address, value);

	return error;

}

nxpTfaBitfield_t tfaContDsc2Bf(nxpTfaDescPtr_t dsc) {
	uint32_t *ptr = (uint32_t *) (&dsc);
	union {
	nxpTfaBitfield_t bf;
	uint32_t num;
	} num_bf;

	num_bf.num = *ptr; 

	return num_bf.bf;
}

enum Tfa98xx_Error tfaContWriteRegsDev(int dev_idx) {
	nxpTfaDeviceList_t *dev = tfaContDevice (dev_idx);
	nxpTfaBitfield_t *bitF;
	int i;
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	if ( !dev ) {
		return Tfa98xx_Error_Bad_Parameter;
	}

	
	for(i=0;i<dev->length;i++) {
		if ( dev->list[i].type == dscPatch ||
			  dev->list[i].type ==dscFile  ||
			  dev->list[i].type ==dscProfile ) break;

		if  ( dev->list[i].type == dscBitfield) {
			bitF = (nxpTfaBitfield_t *)( dev->list[i].offset+(uint8_t *)g_cont);
			err = tfaRunWriteBitfield(dev_idx , *bitF);
		}
		if  ( dev->list[i].type == dscRegister ) {
			err = tfaRunWriteRegister( dev_idx, (nxpTfaRegpatch_t *)
                                                ( dev->list[i].offset+(char*)g_cont));
		}

		if ( err ) break;
	}
	return err;
}

enum Tfa98xx_Error tfaContWriteRegsProf(int dev_idx, int prof_idx) {
	nxpTfaProfileList_t *prof = tfaContProfile( dev_idx, prof_idx);
	nxpTfaBitfield_t *bitf;
	unsigned int i;
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	if ( !prof ) {
		return Tfa98xx_Error_Bad_Parameter;
	}

	if ( tfa98xx_cnt_verbose )
		pr_debug("----- profile: %s (%d) -----\n", tfaContGetString(&prof->name), prof_idx);

	
	for(i=0;i<prof->length;i++) {
		
		if(prof->list[i].type == dscDefault)
			break;

		if  ( prof->list[i].type == dscBitfield) {
			bitf = (nxpTfaBitfield_t *)( prof->list[i].offset+(uint8_t *)g_cont);
			err = tfaRunWriteBitfield(dev_idx , *bitf);
		}
		if  ( prof->list[i].type == dscRegister ) {
			err = tfaRunWriteRegister( dev_idx, (nxpTfaRegpatch_t *)( !prof->list[i].offset+g_cont));
		}
		if ( err ) break;
	}
	return err;
}

enum Tfa98xx_Error tfaContWritePatch(int dev_idx) {
	nxpTfaDeviceList_t *dev = tfaContDevice(dev_idx);
	nxpTfaFileDsc_t *file;
	nxpTfaPatch_t *patchfile;
	int size;

	int i;

	if ( !dev ) {
		return Tfa98xx_Error_Bad_Parameter;
	}
	
	for(i=0;i<dev->length;i++) {
		if ( dev->list[i].type == dscPatch ) {
			file = (nxpTfaFileDsc_t *)(dev->list[i].offset+(uint8_t *)g_cont);
			patchfile =(nxpTfaPatch_t *)&file->data;
			if ( tfa98xx_cnt_verbose ) tfaContShowHeader(&patchfile->hdr);
			size = patchfile->hdr.size - sizeof(nxpTfaPatch_t ); 
			return tfa_dsp_patch(dev_idx,  size, (const unsigned char *) patchfile->data);
		}

	}

	return Tfa98xx_Error_Bad_Parameter; 
}

enum Tfa98xx_Error tfaContWriteFiles(int dev_idx) {
	nxpTfaDeviceList_t *dev = tfaContDevice(dev_idx);
	nxpTfaFileDsc_t *file;
	nxpTfaCmd_t *cmd;
        enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
        char buffer[(MEMTRACK_MAX_WORDS * 3) + 3] = {0}; 
	int i, size = 0;

	if ( !dev ) {
		return Tfa98xx_Error_Bad_Parameter;
	}

	
	for(i=0;i<dev->length;i++) {
		if ( dev->list[i].type == dscFile ) {
			file = (nxpTfaFileDsc_t *)(dev->list[i].offset+(uint8_t *)g_cont);
			if ( tfaContWriteFile(dev_idx,  file, 0 , TFA_MAX_VSTEP_MSG_MARKER) ){
				return Tfa98xx_Error_Bad_Parameter;
			}
		}

		if  ( dev->list[i].type == dscSetInputSelect || 
		      dev->list[i].type == dscSetOutputSelect || 
		      dev->list[i].type == dscSetProgramConfig ||
		      dev->list[i].type == dscSetLagW ||
		      dev->list[i].type == dscSetGains ||
		      dev->list[i].type == dscSetvBatFactors ||
		      dev->list[i].type == dscSetSensesCal ||
		      dev->list[i].type == dscSetSensesDelay ||
		      dev->list[i].type == dscSetMBDrc ) {
			create_dsp_buffer_msg((nxpTfaMsg_t *)
			                      ( dev->list[i].offset+(char*)g_cont), buffer, &size);
			err = tfa_dsp_msg(dev_idx, size, buffer);
			if ( tfa98xx_cnt_verbose ) {
				pr_debug("command: %s=0x%02x%02x%02x \n", 
					tfaContGetCommandString(dev->list[i].type), 
				(unsigned char)buffer[0], (unsigned char)buffer[1], (unsigned char)buffer[2]);
			}
		}

		if  ( dev->list[i].type == dscCmd ) {
			size = *(uint16_t *)(dev->list[i].offset+(char*)g_cont);
			err = tfa_dsp_msg(dev_idx, size,  dev->list[i].offset+2+(char*)g_cont);
			if ( tfa98xx_cnt_verbose ) {
				cmd = (nxpTfaCmd_t *)(dev->list[i].offset+(uint8_t *)g_cont);
				pr_debug("Writing cmd=0x%02x%02x%02x \n", cmd->value[0], cmd->value[1], cmd->value[2]);
			}
		}
		if (err != Tfa98xx_Error_Ok)
			break;

		if  ( dev->list[i].type == dscCfMem ) {
			err = tfaRunWriteDspMem(dev_idx, (nxpTfaDspMem_t *)(dev->list[i].offset+(uint8_t *)g_cont));
		}

		if (err != Tfa98xx_Error_Ok)
			break;
	}

	return err;
}

enum Tfa98xx_Error tfaContWriteFilesProf(int dev_idx, int prof_idx, int vstep_idx) {
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	nxpTfaProfileList_t *prof = tfaContProfile(dev_idx, prof_idx);
	unsigned int i;
	nxpTfaFileDsc_t *file;
	nxpTfaPatch_t *patchfile;
	int size;

	if ( !prof ) {
		return Tfa98xx_Error_Bad_Parameter;
	}

	
	for(i=0;i<prof->length;i++) {
		switch (prof->list[i].type) {
			case dscFile:
				file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)g_cont);
				err = tfaContWriteFile(dev_idx,  file, vstep_idx, TFA_MAX_VSTEP_MSG_MARKER);
				break;
			case dscFilter:
				/* Filters are not written during coldstart
				 * Since calibration, SBSL and many other actions can overwrite them again
				 * They are written during operating mode (tfa_start)
				 */
				
				break;
			case dscPatch:
				file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)g_cont);
				patchfile =(nxpTfaPatch_t *)&file->data;
				if ( tfa98xx_cnt_verbose ) tfaContShowHeader(&patchfile->hdr);
				size = patchfile->hdr.size - sizeof(nxpTfaPatch_t ); 
				err = tfa_dsp_patch(dev_idx,  size, (const unsigned char *) patchfile->data);
				break;
			case dscCfMem:
				err = tfaRunWriteDspMem(dev_idx, (nxpTfaDspMem_t *)(prof->list[i].offset+(uint8_t *)g_cont));
				break;
			default:
				
				break;
		}
	}

	return err;
}

enum Tfa98xx_Error tfaContWriteItem(int dev_idx, nxpTfaDescPtr_t * dsc) {
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	
	nxpTfaRegpatch_t *reg;
	nxpTfaMode_t *cas;
	nxpTfaBitfield_t *bitf;

	switch (dsc->type) {
        case dscDefault:
	case dscDevice: 
	case dscProfile:    
		break;
	case dscRegister:   
		reg = (nxpTfaRegpatch_t *)(dsc->offset+(uint8_t *)g_cont);
		return tfaRunWriteRegister(dev_idx, reg);
		
		break;
	case dscString: 
		pr_debug(";string: %s\n", tfaContGetString(dsc));
		break;
	case dscFile:       
	case dscPatch:
		break;
	case dscMode:
		cas = (nxpTfaMode_t *)(dsc->offset+(uint8_t *)g_cont);
		if(cas->value == Tfa98xx_Mode_RCV)
			tfa98xx_select_mode(dev_idx, Tfa98xx_Mode_RCV);
		else
			tfa98xx_select_mode(dev_idx, Tfa98xx_Mode_Normal);
		break;
	case dscCfMem:
		err = tfaRunWriteDspMem(dev_idx, (nxpTfaDspMem_t *)(dsc->offset+(uint8_t *)g_cont));
		break;
	case dscBitfield:
		bitf = (nxpTfaBitfield_t *)(dsc->offset+(uint8_t *)g_cont);
		return tfaRunWriteBitfield(dev_idx , *bitf);
		break;
	case dscFilter:
		return tfaRunWriteFilter(dev_idx, (nxpTfaContBiquad_t *)(dsc->offset+(uint8_t *)g_cont));
		break;
	}

	return err;
}

static unsigned int tfa98xx_sr_from_field(unsigned int field)
{
	switch (field) {
		case 0:
			return 8000;
		case 1:
			return 11025;
		case 2:
			return 12000;
		case 3:
			return 16000;
		case 4:
			return 22050;
		case 5:
			return 24000;
		case 6:
			return 32000;
		case 7:
			return 44100;
		case 8:
			return 48000;
		default:
			return 0;
	}
}

enum Tfa98xx_Error tfa_write_filters(int dev_idx, int prof_idx) {
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;	
	nxpTfaProfileList_t *prof = tfaContProfile(dev_idx, prof_idx);
	unsigned int i;
	int status;
	
	if ( !prof ) {
		return Tfa98xx_Error_Bad_Parameter;
	}

	if ( tfa98xx_cnt_verbose ) {
		pr_debug("----- profile: %s (%d) -----\n", tfaContGetString(&prof->name), prof_idx);
		pr_debug("Waiting for CLKS... \n");
	}

	for(i=10; i>0; i--) {
		err = tfa98xx_dsp_system_stable(dev_idx, &status);
		if(status)
			break;
		else
			msleep_interruptible(10);
	}

	if(i==0) {
		if ( tfa98xx_cnt_verbose )
			pr_err("Unable to write filters, CLKS=0 \n");
				
		return Tfa98xx_Error_StateTimedOut;	
	}

	
	for(i=0;i<prof->length;i++) {
		if  ( prof->list[i].type == dscFilter ) {
			if (tfaContWriteItem(dev_idx, &prof->list[i]) != Tfa98xx_Error_Ok)
				return Tfa98xx_Error_Bad_Parameter;
		}
	}

	return err;
}

unsigned int tfa98xx_get_profile_sr(int dev_idx, unsigned int prof_idx)
{
	nxpTfaBitfield_t *bitf;
	unsigned int i;
	nxpTfaDeviceList_t *dev;
	nxpTfaProfileList_t *prof;
	int fs_profile = -1;

	dev = tfaContDevice (dev_idx);
	if (!dev)
		return 0;

	prof = tfaContProfile(dev_idx, prof_idx);
	if (!prof)
		return 0;

	
	for(i = 0; i < prof->length; i++) {
		if(prof->list[i].type == dscDefault)
			break;

		
		if ( prof->list[i].type == dscBitfield ) {
			bitf = (nxpTfaBitfield_t *)(prof->list[i].offset+(uint8_t *)g_cont);
			if(bitf->field == TFA_FAM(dev_idx, AUDFS)) {
				fs_profile = bitf->value;
				break;
			}
		}
	}

	pr_debug("%s - profile fs: 0x%x = %dHz (%d - %d)\n", __FUNCTION__, fs_profile,
					tfa98xx_sr_from_field(fs_profile),
					dev_idx, prof_idx);
	if (fs_profile != -1)
		return tfa98xx_sr_from_field(fs_profile);

	
	
	for(i = 0; i < dev->length; i++) {
		if ( dev->list[i].type == dscPatch ||
			  dev->list[i].type ==dscFile  ||
			  dev->list[i].type ==dscProfile ) break;

		if  ( dev->list[i].type == dscBitfield) {
			bitf = (nxpTfaBitfield_t *)( dev->list[i].offset+(uint8_t *)g_cont);
			if(bitf->field == TFA_FAM(dev_idx, AUDFS)) {
				fs_profile = bitf->value;
				break;
			}
		}
		
	}

	pr_debug("%s - default fs: 0x%x = %dHz (%d - %d)\n", __FUNCTION__, fs_profile,
					tfa98xx_sr_from_field(fs_profile),
					dev_idx, prof_idx);
	if (fs_profile != -1)
		return tfa98xx_sr_from_field(fs_profile);

	return 48000;
}

enum Tfa98xx_Error get_sample_rate_info(int dev_idx, nxpTfaProfileList_t *prof, nxpTfaProfileList_t *previous_prof, int fs_previous_profile)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	nxpTfaBitfield_t *bitf;
	unsigned int i;
	int fs_default_profile=8;	
	int fs_next_profile=8;		


	
	for(i=0;i<previous_prof->length;i++) {
		
		if(i == 0) {
			while(previous_prof->list[i].type != dscDefault && i < previous_prof->length) {
				i++;
			}
			i++;
		}

		
		if(i < previous_prof->length) {
			if ( previous_prof->list[i].type == dscBitfield ) {
				bitf = (nxpTfaBitfield_t *)(previous_prof->list[i].offset+(uint8_t *)g_cont);
				if(bitf->field == TFA_FAM(dev_idx, AUDFS)) {
					fs_default_profile = bitf->value;
					break;
				}
			}
		}
	}

	
	for(i=0;i<prof->length;i++) {
		
		if(prof->list[i].type == dscDefault)
			break;
		
		if ( prof->list[i].type == dscBitfield ) {
			bitf = (nxpTfaBitfield_t *)(prof->list[i].offset+(uint8_t *)g_cont);
			if(bitf->field == TFA_FAM(dev_idx, AUDFS)) {
				fs_next_profile = bitf->value;
				break;
			}
		}
	}


	if(fs_next_profile != fs_default_profile) {
		if ( tfa98xx_cnt_verbose )
			pr_debug("Writing delay tables for AUDFS=%d \n", fs_next_profile);

		err = tfa98xx_dsp_write_tables(dev_idx, fs_next_profile);
	} else if(fs_default_profile != fs_previous_profile) {
		if ( tfa98xx_cnt_verbose )
			pr_debug("Writing delay tables for AUDFS=%d \n", fs_default_profile);

		err = tfa98xx_dsp_write_tables(dev_idx, fs_default_profile);
	}

	return err;
}

enum Tfa98xx_Error tfaContWriteProfile(int dev_idx, int prof_idx, int vstep_idx) {
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	nxpTfaProfileList_t *prof = tfaContProfile(dev_idx, prof_idx);
	nxpTfaProfileList_t *previous_prof = tfaContProfile(dev_idx, tfa_get_swprof(dev_idx));
	char buffer[(MEMTRACK_MAX_WORDS * 3) + 3] = {0}; 
	unsigned int i, k=0, j=0, tries=0;
	nxpTfaFileDsc_t *file;
	nxpTfaCmd_t *cmd;
	int size = 0, ready, fs_previous_profile = 8; 

	if ( !prof )
		return Tfa98xx_Error_Bad_Parameter;

	if ( tfa98xx_cnt_verbose ) {
		tfa98xx_trace_printk("device:%s profile:%s vstep:%d\n", tfaContDeviceName(dev_idx),
					tfaContProfileName(dev_idx,prof_idx),vstep_idx);
	}

	
	if (prof->group == previous_prof->group && prof->group != 0) {
		if ( tfa98xx_cnt_verbose ) {
			pr_debug("The new profile (%s) is in the same group as the current profile (%s) \n", 
				tfaContGetString(&prof->name), tfaContGetString(&previous_prof->name));
		}
	} else {
		
		tfaRunMute(dev_idx);

		
		fs_previous_profile = TFA_GET_BF(dev_idx, AUDFS);

		
		if(tfa98xx_dev_family(dev_idx) == 2) {
			TFA_SET_BF_VOLATILE(dev_idx, SBSL, 0);
		}

			err = tfa98xx_powerdown(dev_idx, 1);
			if (err) return err;
			
			
			do {
				err = tfa98xx_dsp_system_stable(dev_idx, &ready);
				if (!ready)
					break;
				else
					msleep_interruptible(10); 
				tries++;
			} while (tries <= 100);

			if (tries > 100) {
				pr_debug("Wait for PLL powerdown timed out!\n");
				return Tfa98xx_Error_StateTimedOut;
			}
	}

	
	
	if (tfa98xx_cnt_verbose) {
		pr_debug("---------- default settings profile: %s (%d) ---------- \n", 
				tfaContGetString(&previous_prof->name), tfa_get_swprof(dev_idx));

		if(tfa98xx_dev_family(dev_idx) == 2)
			err = show_current_state(dev_idx);
	}

	
	for(i=0;i<previous_prof->length;i++) {
		
		if(i == 0) {
			while(previous_prof->list[i].type != dscDefault && i < previous_prof->length) {
				i++;
			}
			i++;
		}

		
		if(i < previous_prof->length) {
			if ( tfaContWriteItem(dev_idx,  &previous_prof->list[i]) != Tfa98xx_Error_Ok )
				return Tfa98xx_Error_Bad_Parameter;
		}
	}

	if ( tfa98xx_cnt_verbose )
		pr_debug("---------- new settings profile: %s (%d) ---------- \n", 
				tfaContGetString(&prof->name), prof_idx);

	
	for(i=0;i<prof->length;i++) {
		
		j = i;

		
		
		switch (prof->list[i].type) {
			case dscFile:
			case dscPatch:
			case dscSetInputSelect:
			case dscSetOutputSelect:
			case dscSetProgramConfig:
			case dscSetLagW:
			case dscSetGains:
			case dscSetvBatFactors:
			case dscSetSensesCal:
			case dscSetSensesDelay:
			case dscSetMBDrc:
			case dscCmd:
			case dscFilter:
			case dscDefault:
				
				i = prof->length;
				break;
			default:
				err = tfaContWriteItem(dev_idx,  &prof->list[i]);
				if ( err != Tfa98xx_Error_Ok )
					return Tfa98xx_Error_Bad_Parameter;
				break;
		}
	}

	if (prof->group != previous_prof->group || prof->group == 0) {
		if(tfa98xx_dev_family(dev_idx) == 2)
			TFA_SET_BF_VOLATILE(dev_idx, MANSCONF, 1);

		
		err = tfa_cf_powerup(dev_idx);
		if (err) return err;
		if (tfa98xx_cnt_verbose && tfa98xx_dev_family(dev_idx) == 2)
			err = show_current_state(dev_idx);

		if (tfa98xx_dev_family(dev_idx) == 2) {
			
			TFA_SET_BF_VOLATILE(dev_idx, SBSL, 0);
            
            if (TFA_GET_BF(dev_idx, REFCKSEL) == 1)
                TFA_SET_BF(dev_idx, RST, 0);
		}
	}

	
	err = get_sample_rate_info(dev_idx, prof, previous_prof, fs_previous_profile);
	if (err) return err;

	if(tfa98xx_dev_family(dev_idx) == 2) {
		for(i=0;i<previous_prof->length;i++) {
			
			if(i == 0) {
				while(previous_prof->list[i].type != dscDefault && i < previous_prof->length) {
					i++;
				}
				i++;
			}

			
			if(i < previous_prof->length) {
				if(previous_prof->list[i].type == dscFile || previous_prof->list[i].type == dscPatch) {
					
					if ( tfa98xx_cnt_verbose && k==0) {
						pr_debug("---------- files default profile: %s (%d) ---------- \n", 
								tfaContGetString(&previous_prof->name), prof_idx);
						k++;
					}
					file = (nxpTfaFileDsc_t *)(previous_prof->list[i].offset+(uint8_t *)g_cont);
					err = tfaContWriteFile(dev_idx,  file, vstep_idx, TFA_MAX_VSTEP_MSG_MARKER);
				}
			}
		}
	}

	if ( tfa98xx_cnt_verbose) {
		pr_debug("---------- files new profile: %s (%d) ---------- \n", 
				tfaContGetString(&prof->name), prof_idx);
	}

	for(i=j;i<prof->length;i++) {
		
		if(prof->list[i].type == dscDefault)
			break;

		switch (prof->list[i].type) {
			case dscFile:
			case dscPatch:
				file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)g_cont);
				err = tfaContWriteFile(dev_idx, file, vstep_idx, TFA_MAX_VSTEP_MSG_MARKER);
				break;
			case dscSetInputSelect:
			case dscSetOutputSelect:
			case dscSetProgramConfig:
			case dscSetLagW:
			case dscSetGains:
			case dscSetvBatFactors:
			case dscSetSensesCal:
			case dscSetSensesDelay:
			case dscSetMBDrc:
				create_dsp_buffer_msg((nxpTfaMsg_t *)
					( prof->list[i].offset+(char*)g_cont), buffer, &size);
				err = tfa_dsp_msg(dev_idx, size, buffer);

				if ( tfa98xx_cnt_verbose )
					pr_debug("command: %s=0x%02x%02x%02x \n", 
						tfaContGetCommandString(prof->list[i].type), 
					(unsigned char)buffer[0], (unsigned char)buffer[1], (unsigned char)buffer[2]);
				break;
			case dscCmd:
				size = *(uint16_t *)(prof->list[i].offset+(char*)g_cont);
				err = tfa_dsp_msg(dev_idx, size, prof->list[i].offset+2+(char*)g_cont);			

				if ( tfa98xx_cnt_verbose ) {
					cmd = (nxpTfaCmd_t *)(prof->list[i].offset+(uint8_t *)g_cont);
					pr_debug("Writing cmd=0x%02x%02x%02x \n", cmd->value[0], cmd->value[1], cmd->value[2]);
				}
				break;
			default:
				
				if (tfaContWriteItem(dev_idx,  &prof->list[i]) != Tfa98xx_Error_Ok)
					return Tfa98xx_Error_Bad_Parameter;
				break;
		}

		if (err != Tfa98xx_Error_Ok)
			return err;
	}

	if ((prof->group != previous_prof->group || prof->group == 0) && tfa98xx_dev_family(dev_idx) == 2) {
		if (TFA_GET_BF(dev_idx, REFCKSEL) == 0) {		
					
			TFA_SET_BF_VOLATILE(dev_idx, SBSL, 1);
		}
	}

	return err;
}

enum Tfa98xx_Error tfaContWriteFilesVstep(int dev_idx, int prof_idx, int vstep_idx)
{
	nxpTfaProfileList_t *prof = tfaContProfile(dev_idx, prof_idx);
	unsigned int i;
	nxpTfaFileDsc_t *file;
	nxpTfaHeader_t *hdr;
	nxpTfaHeaderType_t type;
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;

	if ( !prof )
		return Tfa98xx_Error_Bad_Parameter;

	if ( tfa98xx_cnt_verbose )
		tfa98xx_trace_printk("device:%s profile:%s vstep:%d\n", tfaContDeviceName(dev_idx),
					tfaContProfileName(dev_idx,prof_idx),vstep_idx);

	
	for(i=0;i<prof->length;i++) {
		if ( prof->list[i].type == dscFile ) {
			file = (nxpTfaFileDsc_t *)(prof->list[i].offset+(uint8_t *)g_cont);
			hdr = (nxpTfaHeader_t *)file->data;
			type = (nxpTfaHeaderType_t) hdr->id;

			switch (type) {
			case volstepHdr:
				if ( tfaContWriteFile(dev_idx, file, vstep_idx, TFA_MAX_VSTEP_MSG_MARKER) )
					return Tfa98xx_Error_Bad_Parameter;
				break;
			default:
				break;
			}
		}
	}

	return err;
}


char *tfaContGetString(nxpTfaDescPtr_t * dsc)
{
	if ( dsc->type != dscString)
		return nostring;

	return dsc->offset+(char*)g_cont;
}

void individual_calibration_results(Tfa98xx_handle_t handle)
{
	int value_P, value_S;

	
	tfa98xx_dsp_read_mem(handle, 529, 1, &value_P);  
	tfa98xx_dsp_read_mem(handle, 530, 1, &value_S);  
	
	if(value_P != 1 && value_S != 1)
		pr_debug("Calibration failed on both channels! \n");
	else if(value_P != 1) {
		pr_debug("Calibration failed on Primary (Left) channel! \n");
		TFA_SET_BF_VOLATILE(handle, SSLEFTE, 0); 
	}
	else if(value_S != 1) {
		pr_debug("Calibration failed on Secondary (Right) channel! \n");
		TFA_SET_BF_VOLATILE(handle, SSRIGHTE, 0); 
	}

	TFA_SET_BF_VOLATILE(handle, AMPINSEL, 0); 
	TFA_SET_BF_VOLATILE(handle, SBSL, 1); 
}

char *tfaContGetCommandString(uint32_t type)
{
	if(type == dscSetInputSelect)
		return "setInputSelector";
	else if(type == dscSetOutputSelect)
		return "setOutputSelector";
	else if(type == dscSetProgramConfig)
		return "setProgramConfig";
	else if(type == dscSetLagW)
		return "setLagW";
	else if(type == dscSetGains)
		return "setGains";
	else if(type == dscSetvBatFactors)
		return "setvBatFactors";
	else if(type == dscSetSensesCal)
		return "setSensesCal";
	else if(type == dscSetSensesDelay)
		return "setSensesDelay";
	else if(type == dscSetMBDrc)
		return "setMBDrc";
	else if(type == dscFilter)
		return "filter";
	else
		return nostring;
}

char  *tfaContDeviceName(int dev_idx) {
	nxpTfaDeviceList_t *dev;

	if ( dev_idx >= tfa98xx_cnt_max_device() )
		return errorname;

	if ( (dev = tfaContDevice(dev_idx)) == NULL )
		return errorname;

	return tfaContGetString(&dev->name);
}

int tfa_cnt_get_app_name(char *name) {
	unsigned int i;
	int len;

	for(i=0;i<sizeof(g_cont->application);i++) {
		if (isalpha(g_cont->application[i])) 
			name[i] = g_cont->application[i];
		if (g_cont->application[i]=='\0') {
			len = i;
			name[i] = '\0';
			break;
		}
	}
	len = i;
	name[i] = '\0';

	return len;
}

int tfaContGetCalProfile(int dev_idx) {
	int prof, nprof, cal_idx = -2;

	if ( (dev_idx < 0) || (dev_idx >= tfa98xx_cnt_max_device()) )
		return -1;
	
	nprof = tfaContMaxProfile(dev_idx);
	
	for (prof = 0; prof < nprof; prof++) {
		if(strstr(tfaContProfileName(dev_idx, prof), ".cal") != NULL) {
			cal_idx = prof;
			pr_debug("Using calibration profile: '%s'\n", tfaContProfileName(dev_idx, prof));
			break;
		}
	}
	return cal_idx;
}

int tfaContIsTapProfile(int dev_idx, int prof_idx) {
	int nprof;

	if ( (dev_idx < 0) || (dev_idx >= tfa98xx_cnt_max_device()) )
		return -1;
	
	nprof = tfaContMaxProfile(dev_idx);
	
    if(strstr(tfaContProfileName(dev_idx, prof_idx), ".tap") != NULL) {
	    pr_debug("Using Tap profile: '%s'\n", tfaContProfileName(dev_idx, prof_idx));
        return 1;
	}
    
    return 0;
}

char *tfaContProfileName(int dev_idx, int prof_idx) {
	nxpTfaProfileList_t *prof;

	if ( (dev_idx < 0) || (dev_idx >= tfa98xx_cnt_max_device()) )
		return errorname;
	if ( (prof_idx < 0) || (prof_idx >= tfaContMaxProfile(dev_idx)) )
		return nonename;

	
	prof=tfaContGetDevProfList(g_cont, dev_idx, prof_idx);
	return tfaContGetString(&prof->name);
}

nxpTfaProfileList_t *tfaContGet1stProfList(nxpTfaContainer_t * cont)
{
	nxpTfaProfileList_t *prof;
	uint8_t *b = (uint8_t *) cont;

	int maxdev = 0;
	nxpTfaDeviceList_t *dev;

	
	maxdev = cont->ndev;
	
	dev = tfaContGetDevList(cont, maxdev - 1);
        if(dev == NULL)
                return NULL;
        
	b = (uint8_t *) dev + sizeof(nxpTfaDeviceList_t) + dev->length * (sizeof(nxpTfaDescPtr_t));
	prof = (nxpTfaProfileList_t *) b;
	return prof;
}

nxpTfaLiveDataList_t *tfaContGet1stLiveDataList(nxpTfaContainer_t * cont)
{
        nxpTfaLiveDataList_t *ldata;
        nxpTfaProfileList_t *prof;
        nxpTfaDeviceList_t *dev;
	uint8_t *b = (uint8_t *) cont;
	int maxdev, maxprof;
	
	
	maxdev = cont->ndev;
        
	maxprof = cont->nprof;
	
         
	dev = tfaContGetDevList(cont, maxdev - 1);
        
	b = (uint8_t *) dev + sizeof(nxpTfaDeviceList_t) + 
                dev->length * (sizeof(nxpTfaDescPtr_t));

        while(maxprof != 0) {
                
	        prof = (nxpTfaProfileList_t *) b; 
                b += sizeof(nxpTfaProfileList_t) + 
                     ((prof->length-1) * (sizeof(nxpTfaDescPtr_t)));
                maxprof--;
        }

        
        b += 4; 

	ldata = (nxpTfaLiveDataList_t *) b;
	return ldata;
}


enum Tfa98xx_Error tfaContOpen(int dev_idx)
{
	return tfa98xx_open((Tfa98xx_handle_t)dev_idx);
}

enum Tfa98xx_Error tfaContClose(int dev_idx)
{
	return tfa98xx_close(dev_idx);
}

int tfa98xx_cnt_max_device(void) {
	return g_cont !=NULL ? g_cont->ndev : 0;
}

int tfa98xx_cnt_slave2idx(int slave_addr) {
	int idx;

	for(idx=0;idx<g_devs;idx++) {
		if (g_dev[idx]->dev == slave_addr )
			return idx;
	}

	return -1;
}

int tfa98xx_cnt_slave2revid(int slave_addr) {
	int idx = tfa98xx_cnt_slave2idx(slave_addr);
	uint16_t revid;

	if (idx<0)
		return idx;

	
	revid = tfa98xx_get_device_revision(idx);

	
	return (revid&0xFF) >= 0x12 ? revid : -1 ;
}

nxpTfaDeviceList_t *tfaContDevice(int dev_idx) {
	if(dev_idx < g_devs)
		return g_dev[dev_idx];
	
	return NULL;
}

int tfaContMaxProfile(int dev_idx) {
	if ( dev_idx >= g_devs) {
		
		return 0;
	}
	return g_profs[dev_idx];
}

nxpTfaProfileList_t* tfaContNextProfile(nxpTfaProfileList_t* prof) {
	uint8_t *this, *next; 
	nxpTfaProfileList_t* nextprof;
	int listlength; 

        if(prof == NULL)
                return NULL;

	if (prof->ID != TFA_PROFID)
		return NULL;	

	this = (uint8_t *)prof;
	
	listlength = (prof->length - 1)*sizeof(nxpTfaDescPtr_t);
	
	next = this + listlength + sizeof(nxpTfaProfileList_t);
	nextprof = (nxpTfaProfileList_t *)next;

	if (nextprof->ID != TFA_PROFID)
		return NULL;

	return nextprof;
}

nxpTfaLiveDataList_t* tfaContNextLiveData(nxpTfaLiveDataList_t* livedata) {
	nxpTfaLiveDataList_t* nextlivedata = (nxpTfaLiveDataList_t *)( (char*)livedata + (livedata->length*4) + 
                                                                        sizeof(nxpTfaLiveDataList_t) -4);

	if (nextlivedata->ID == TFA_LIVEDATAID)
		return nextlivedata;

	return NULL;
}

nxpTfaProfileList_t* tfaContProfile(int dev_idx, int prof_ipx) {
	if ( dev_idx >= g_devs) {
		
		return NULL;
	}
	if ( prof_ipx >= g_profs[dev_idx]) {
		
		return NULL;
	}

		return g_prof[dev_idx][prof_ipx];
}

int tfaContCrcCheckContainer(nxpTfaContainer_t *cont) {
	uint8_t *base;
	size_t size;
	uint32_t crc;

	base = (uint8_t *)&cont->CRC + 4; 
	size = (size_t)(cont->size - (base - (uint8_t *)cont)); 
	crc = ~crc32_le(~0u, base, size);

	return crc != cont->CRC;
}

void create_dsp_buffer_msg(nxpTfaMsg_t *msg, char *buffer, int *size) 
{
        int i, j = 0;

        
        buffer[0] = msg->cmdId[2];
        buffer[1] = msg->cmdId[1];
        buffer[2] = msg->cmdId[0];

        
        for(i=3; i<3+(msg->msg_size*3); i++) {
                buffer[i] = (uint8_t) ((msg->data[j] >> 16) & 0xffff);
                i++;
                buffer[i] = (uint8_t) ((msg->data[j] >> 8) & 0xff);
                i++;
                buffer[i] = (uint8_t) (msg->data[j] & 0xff);
                j++;
        }

        *size = (3+(msg->msg_size*3)) * sizeof(char);
}

void get_all_features_from_cnt(Tfa98xx_handle_t dev_idx, int *hw_feature_register, int sw_feature_register[2]) {
        nxpTfaFeatures_t *features;
        int i;

        nxpTfaDeviceList_t *dev = tfaContDevice(dev_idx);
  
        
        *hw_feature_register = -1;
        sw_feature_register[0] = -1;
        sw_feature_register[1] = -1;

        if(dev == NULL)
                return;

	
	for(i=0;i<dev->length;i++) {
		if (dev->list[i].type == dscFeatures) {
                        features = (nxpTfaFeatures_t *)(dev->list[i].offset+(uint8_t *)g_cont);
                        *hw_feature_register = features->value[0];
                        sw_feature_register[0] = features->value[1];
                        sw_feature_register[1] = features->value[2];
			break;
		}
	}
}

void get_hw_features_from_cnt(Tfa98xx_handle_t dev_idx, int *hw_feature_register) 
{
        int sw_feature_register[2]; 
        get_all_features_from_cnt(dev_idx, hw_feature_register, sw_feature_register);
}

void get_sw_features_from_cnt(Tfa98xx_handle_t dev_idx, int sw_feature_register[2]) 
{
        int hw_feature_register; 
        get_all_features_from_cnt(dev_idx, &hw_feature_register, sw_feature_register);
}

void tfa_factory_trimmer(Tfa98xx_handle_t dev_idx) 
{
	unsigned short currentValue, delta;
	int result;

	
	
	result = TFA_GET_BF(dev_idx, DCMCCAPI);
	if (result) {
		
		currentValue = (unsigned short)TFA_GET_BF(dev_idx, DCMCC);
		delta = (unsigned short)TFA_GET_BF(dev_idx, USERDEF);

		
		result = TFA_GET_BF(dev_idx, DCMCCSB);
		if (result == 0) {
			
			if(currentValue + delta < 15) {
				TFA_SET_BF_VOLATILE(dev_idx, DCMCC, currentValue + delta);
				if (tfa98xx_cnt_verbose)
					pr_debug("Max coil current is set to: %d \n", currentValue + delta);
			} else {
				TFA_SET_BF_VOLATILE(dev_idx, DCMCC, 15);
				if (tfa98xx_cnt_verbose)
					pr_debug("Max coil current is set to: 15 \n");
			}
		} else if (result == 1) {
			
			if(currentValue - delta > 0) {
				TFA_SET_BF_VOLATILE(dev_idx, DCMCC, currentValue - delta);
				if (tfa98xx_cnt_verbose)
					pr_debug("Max coil current is set to: %d \n", currentValue - delta);
			} else {
				TFA_SET_BF_VOLATILE(dev_idx, DCMCC, 0);
				if (tfa98xx_cnt_verbose)
					pr_debug("Max coil current is set to: 0 \n");
			}
		}
	}
}

enum Tfa98xx_Error tfa_set_filters(int dev_idx, int prof_idx) {
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	nxpTfaProfileList_t *prof = tfaContProfile(dev_idx, prof_idx);
	unsigned int i;

	if ( !prof )
		return Tfa98xx_Error_Bad_Parameter;

	
	if (TFA_GET_BF(dev_idx, PWDN) == 1)
		return Tfa98xx_Error_Ok;

	
	for(i=0;i<prof->length;i++) {
		
		if(prof->list[i].type == dscDefault)
			break;

		
		if ( prof->list[i].type == dscFilter) {
			if (tfaContWriteItem(dev_idx,  &prof->list[i]) != Tfa98xx_Error_Ok)
				return err;
		}
	}

	return err;
}

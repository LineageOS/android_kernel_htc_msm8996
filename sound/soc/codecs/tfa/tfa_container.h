
#ifndef TFACONTAINER_H_
#define TFACONTAINER_H_

#define TFACONT_MAXDEVS  (4)   
#define TFACONT_MAXPROFS (16) 

#include "tfa98xx_parameters.h"

enum tfa_error tfa_load_cnt(void *cnt, int length);

void tfa_deinit(void);

void individual_calibration_results(Tfa98xx_handle_t handle);

char *tfaContGetString(nxpTfaDescPtr_t *dsc); 

char *tfaContGetCommandString(uint32_t type);

int tfa_cnt_get_devid(nxpTfaContainer_t *cnt, int dev_idx) ;

int tfa98xx_cnt_max_device(void);

void tfa_cnt_verbose(int level);

nxpTfaContainer_t* tfa98xx_get_cnt(void);

int tfa98xx_cnt_slave2idx(int slave_addr);

int tfa98xx_cnt_slave2revid(int slave_addr);

enum Tfa98xx_Error tfaContGetSlave(int dev_idx, uint8_t *slave_addr);

enum Tfa98xx_Error tfaContWriteRegsDev(int dev_idx);

enum Tfa98xx_Error tfaContWriteRegsProf(int dev_idx, int prof_idx);

enum Tfa98xx_Error tfaContWritePatch(int dev_idx);

enum Tfa98xx_Error tfaContWriteFiles(int dev_idx);

unsigned int tfa98xx_get_profile_sr(int dev_idx, unsigned int prof_idx);

enum Tfa98xx_Error tfaContOpen(int dev_idx);

enum Tfa98xx_Error tfaContClose(int dev_idx);

char  *tfaContDeviceName(int dev_idx);

int tfa_cnt_get_app_name(char *name);

int tfaContGetCalProfile(int dev_idx);

int tfaContIsTapProfile(int dev_idx, int prof_idx);

char  *tfaContProfileName(int dev_idx, int prof_idx);

int tfaContMaxProfile(int dev_idx);

enum Tfa98xx_Error tfaContWriteProfile(int dev_idx, int prof_idx, int vstep_idx);

void tfa98xx_set_spkr_select(Tfa98xx_handle_t dev_idx, char *configuration);

void tfaContSetCurrentVstep(int channel, int vstep_idx);

int tfaContGetCurrentVstep(int channel);

enum Tfa98xx_Error tfa_cont_write_filterbank(int dev_idx, nxpTfaFilter_t *filter);

enum Tfa98xx_Error tfaContWriteFilesProf(int dev_idx, int prof_idx, int vstep_idx);


enum Tfa98xx_Error tfaContWriteFilesVstep(int dev_idx, int prof_idx, int vstep_idx);
enum Tfa98xx_Error tfaContWriteDrcFile(int dev_idx, int size, uint8_t data[]);

nxpTfaDeviceList_t *tfaContGetDevList(nxpTfaContainer_t * cont, int dev_idx);

nxpTfaProfileList_t *tfaContGetDevProfList(nxpTfaContainer_t *cont, int dev_idx, int prof_idx);

nxpTfaLiveDataList_t *tfaContGetDevLiveDataList(nxpTfaContainer_t *cont, int dev_idx, int livedata_idx);

int tfaContCrcCheckContainer(nxpTfaContainer_t *cont);

nxpTfaDeviceList_t *tfaContDevice(int dev_idx);

nxpTfaProfileList_t* tfaContProfile(int dev_idx, int prof_ipx);

nxpTfaProfileList_t *tfaContGet1stProfList(nxpTfaContainer_t *cont);

nxpTfaProfileList_t* tfaContNextProfile(nxpTfaProfileList_t *prof);

nxpTfaLiveDataList_t *tfaContGet1stLiveDataList(nxpTfaContainer_t *cont);

nxpTfaLiveDataList_t* tfaContNextLiveData(nxpTfaLiveDataList_t *livedata_idx);

enum Tfa98xx_Error tfaRunWriteBitfield(Tfa98xx_handle_t dev_idx,  nxpTfaBitfield_t bf);

enum Tfa98xx_Error tfaContWriteFile(int dev_idx,  nxpTfaFileDsc_t *file, int vstep_idx, int vstep_msg_idx);

int tfacont_get_max_vstep(int dev_idx, int prof_idx);

nxpTfaFileDsc_t *tfacont_getfiledata(int dev_idx, int prof_idx, enum nxpTfaHeaderType type);

void tfaContShowHeader(nxpTfaHeader_t *hdr);

enum Tfa98xx_Error tfaRunReadBitfield(Tfa98xx_handle_t dev_idx,  nxpTfaBitfield_t *bf);

void get_hw_features_from_cnt(Tfa98xx_handle_t dev_idx, int *hw_feature_register);

void get_sw_features_from_cnt(Tfa98xx_handle_t dev_idx, int sw_feature_register[2]);

void tfa_factory_trimmer(Tfa98xx_handle_t dev_idx);

enum Tfa98xx_Error tfa_set_filters(int dev_idx, int prof_idx);

#endif 

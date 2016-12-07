
#ifndef __TFA_INTERNAL_H__
#define __TFA_INTERNAL_H__

#include "tfa_dsp_fw.h"
#include "tfa_service.h"
#include "config.h"

#if __GNUC__ >= 4
  #define TFA_INTERNAL __attribute__ ((visibility ("hidden")))
#else
  #define TFA_INTERNAL
#endif


#define TFA98XX_GENERIC_SLAVE_ADDRESS 0x1C

enum featureSupport {
	supportNotSet, 
	supportNo,
	supportYes
};

typedef enum featureSupport featureSupport_t;


struct tfa98xx_control {
	bool deferrable;
	bool triggered;
	int wr_value;
	int rd_value;
	bool rd_valid;
};

struct tfa98xx_controls {
	struct tfa98xx_control otc;
	struct tfa98xx_control mtpex;
	struct tfa98xx_control calib;
};

struct tfa_device_ops {
	enum Tfa98xx_Error (*tfa_init)(Tfa98xx_handle_t dev_idx);
	enum Tfa98xx_Error (*tfa_dsp_reset)(Tfa98xx_handle_t dev_idx, int state);
	enum Tfa98xx_Error (*tfa_dsp_system_stable)(Tfa98xx_handle_t handle, int *ready);
	enum Tfa98xx_Error (*tfa_dsp_write_tables)(Tfa98xx_handle_t dev_idx, int sample_rate);
	struct tfa98xx_controls controls;
};

struct Tfa98xx_handle_private {
	int in_use;
	int buffer_size;
	unsigned char slave_address;
	unsigned short rev;
	unsigned char tfa_family; 
	enum featureSupport supportDrc;
	enum featureSupport supportFramework;
	enum featureSupport support_saam;
	int sw_feature_bits[2]; 
	int hw_feature_bits; 
	int profile;	
	int vstep[2]; 
	unsigned char spkr_count;
	unsigned char spkr_select;
	unsigned char support_tcoef;
	enum Tfa98xx_DAI daimap;
	int mohm[3]; 
	struct tfa_device_ops dev_ops;
	uint16_t interrupt_enable[3];
	uint16_t interrupt_status[3];
};

extern TFA_INTERNAL struct Tfa98xx_handle_private handles_local[];
TFA_INTERNAL int tfa98xx_handle_is_open(Tfa98xx_handle_t h);
TFA_INTERNAL enum Tfa98xx_Error tfa98xx_check_rpc_status(Tfa98xx_handle_t handle, int *pRpcStatus);
TFA_INTERNAL enum Tfa98xx_Error tfa98xx_wait_result(Tfa98xx_handle_t handle, int waitRetryCount);
TFA_INTERNAL void tfa98xx_apply_deferred_calibration(Tfa98xx_handle_t handle);
TFA_INTERNAL void tfa98xx_deferred_calibration_status(Tfa98xx_handle_t handle, int calibrateDone);
TFA_INTERNAL int print_calibration(Tfa98xx_handle_t handle, char *str, size_t size);

#endif 


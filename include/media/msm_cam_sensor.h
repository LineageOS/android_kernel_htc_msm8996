#ifndef __LINUX_MSM_CAM_SENSOR_H
#define __LINUX_MSM_CAM_SENSOR_H

#include <uapi/media/msm_cam_sensor.h>

#include <linux/compat.h>

#define MAX_ACT_NAME_SIZE_32 32
#define LC898214_HEX_MAX_32 0x7FFF 
#define LC898214_HEX_MIN_32 0x8001 
#define LC898214_DEC_MAX_32 1023

struct fuse_id32{
	uint32_t fuse_id_word1;
	uint32_t fuse_id_word2;
	uint32_t fuse_id_word3;
	uint32_t fuse_id_word4;
};

typedef struct{
	char    ACT_NAME[MAX_ACT_NAME_SIZE_32]; 
	uint8_t VCM_START_MSB;
	uint8_t VCM_START_LSB;
	uint8_t AF_INF_MSB;
	uint8_t AF_INF_LSB;
	uint8_t AF_MACRO_MSB;
	uint8_t AF_MACRO_LSB;
	uint8_t VCM_BIAS;
	uint8_t VCM_OFFSET;
	uint8_t VCM_BOTTOM_MECH_MSB;
	uint8_t VCM_BOTTOM_MECH_LSB;
	uint8_t VCM_TOP_MECH_MSB;
	uint8_t VCM_TOP_MECH_LSB;
	uint8_t VCM_VENDOR_ID_VERSION;
	uint8_t VCM_VENDOR;
	uint8_t ACT_ID;
	uint32_t MODULE_ID_AB;
}af_value_t32;

struct pixel_tt32 {
	int   x;
	int   y;
};

struct pixels_array_tt32 {
	struct pixel_tt32 pix[40];
	int	count;
};

enum actuator_I2C_func_select32 {
        WRITE_SEQ_TABLE_32,
        WRITE_TABLE_W_MICRODELAY_32,
        WRITE_MULTI_TABLE_32
};

struct msm_actuator_af_OTP_info_t32 {
        uint8_t VCM_OTP_Read;
        uint16_t VCM_Start;
        uint16_t VCM_Infinity;
        uint16_t VCM_Macro;
        
        uint8_t VCM_Bias;
        uint8_t VCM_Offset;
        uint16_t VCM_Bottom_Mech;
        uint16_t VCM_Top_Mech;
        uint8_t VCM_Vendor_Id_Version;
        
        uint8_t VCM_Vendor;
        uint8_t act_id;
        char act_name[MAX_SENSOR_NAME];
        uint32_t MODULE_ID_AB;
};


#ifdef CONFIG_COMPAT

struct msm_sensor_power_setting32 {
	enum msm_sensor_power_seq_type_t seq_type;
	uint16_t seq_val;
	compat_uint_t config_val;
	uint16_t delay;
	compat_uptr_t data[10];
};

struct msm_sensor_power_setting_array32 {
	struct msm_sensor_power_setting32 power_setting_a[MAX_POWER_CONFIG];
	compat_uptr_t power_setting;
	uint16_t size;
	struct msm_sensor_power_setting32
		power_down_setting_a[MAX_POWER_CONFIG];
	compat_uptr_t power_down_setting;
	uint16_t size_down;
};

struct msm_camera_sensor_slave_info32 {
	char sensor_name[32];
	char eeprom_name[32];
	char actuator_name[32];
	char ois_name[32];
	char flash_name[32];
	enum msm_sensor_camera_id_t camera_id;
	uint16_t slave_addr;
	enum i2c_freq_mode_t i2c_freq_mode;
	enum msm_camera_i2c_reg_addr_type addr_type;
	struct msm_sensor_id_info_t sensor_id_info;
	struct msm_sensor_power_setting_array32 power_setting_array;
	uint8_t  is_init_params_valid;
	struct msm_sensor_init_params sensor_init_params;
	enum msm_sensor_output_format_t output_format;
};

struct msm_camera_csid_lut_params32 {
	uint8_t num_cid;
	struct msm_camera_csid_vc_cfg vc_cfg_a[MAX_CID];
	compat_uptr_t vc_cfg[MAX_CID];
};

struct msm_camera_csid_params32 {
	uint8_t lane_cnt;
	uint16_t lane_assign;
	uint8_t phy_sel;
	uint32_t csi_clk;
	struct msm_camera_csid_lut_params32 lut_params;
	uint8_t csi_3p_sel;
};

struct msm_camera_csi2_params32 {
	struct msm_camera_csid_params32 csid_params;
	struct msm_camera_csiphy_params csiphy_params;
	uint8_t csi_clk_scale_enable;
};

struct csid_cfg_data32 {
	enum csid_cfg_type_t cfgtype;
	union {
		uint32_t csid_version;
		compat_uptr_t csid_params;
		compat_uptr_t csid_testmode_params;
	} cfg;
};

struct eeprom_read_t32 {
	compat_uptr_t dbuffer;
	uint32_t num_bytes;
};

struct eeprom_write_t32 {
	compat_uptr_t dbuffer;
	uint32_t num_bytes;
};

struct msm_eeprom_info_t32 {
	compat_uptr_t power_setting_array;
	enum i2c_freq_mode_t i2c_freq_mode;
	compat_uptr_t mem_map_array;
};

struct msm_eeprom_cfg_data32 {
	enum eeprom_cfg_type_t cfgtype;
	uint8_t is_supported;
	union {
		char eeprom_name[MAX_SENSOR_NAME];
		struct eeprom_get_t get_data;
		struct eeprom_read_t32 read_data;
		struct eeprom_write_t32 write_data;
		struct msm_eeprom_info_t32 eeprom_info;
	} cfg;
};

struct msm_camera_i2c_seq_reg_setting32 {
	compat_uptr_t reg_setting;
	uint16_t size;
	enum msm_camera_i2c_reg_addr_type addr_type;
	uint16_t delay;
};

struct msm_camera_i2c_reg_setting32 {
	compat_uptr_t reg_setting;
	uint16_t size;
	enum msm_camera_i2c_reg_addr_type addr_type;
	enum msm_camera_i2c_data_type data_type;
	uint16_t delay;
};

struct msm_camera_i2c_array_write_config32 {
	struct msm_camera_i2c_reg_setting32 conf_array;
	uint16_t slave_addr;
};

struct msm_actuator_tuning_params_t32 {
	int16_t initial_code;
	uint16_t pwd_step;
	uint16_t region_size;
	uint32_t total_steps;
	compat_uptr_t region_params;
};

struct msm_actuator_params_t32 {
	enum actuator_type act_type;
	uint8_t reg_tbl_size;
	uint16_t data_size;
	uint16_t init_setting_size;
	uint32_t i2c_addr;
	enum i2c_freq_mode_t i2c_freq_mode;
	enum msm_camera_i2c_reg_addr_type i2c_addr_type;
	enum msm_camera_i2c_data_type i2c_data_type;
	compat_uptr_t reg_tbl_params;
	compat_uptr_t init_settings;
	struct park_lens_data_t park_lens;
	
	char ACT_NAME[MAX_ACT_NAME_SIZE_32];
	
};

struct msm_actuator_set_info_t32 {
	struct msm_actuator_params_t32 actuator_params;
	struct msm_actuator_tuning_params_t32 af_tuning_params;
	uint8_t enable_focus_step_log;
	compat_uptr_t step_position_table;                
	enum actuator_I2C_func_select32 act_i2c_select; 
};

struct sensor_init_cfg_data32 {
	enum msm_sensor_init_cfg_type_t cfgtype;
	struct msm_sensor_info_t        probed_info;
	char                            entity_name[MAX_SENSOR_NAME];
	union {
		compat_uptr_t setting;
	} cfg;
};

struct msm_actuator_move_params_t32 {
	int8_t dir;
	int8_t sign_dir;
	int16_t dest_step_pos;
	int32_t num_steps;
	uint16_t curr_lens_pos;
	compat_uptr_t ringing_params;
};

struct msm_actuator_cfg_data32 {
	int cfgtype;
	uint8_t is_af_supported;
	
	uint32_t ois_gain;
	
	union {
		struct msm_actuator_move_params_t32 move;
		struct msm_actuator_set_info_t32 set_info;
		struct msm_actuator_get_info_t get_info;
		struct msm_actuator_set_position_t setpos;
		enum af_camera_name cam_name;
		
		af_value_t32 af_value;
		
	} cfg;
};

struct csiphy_cfg_data32 {
	enum csiphy_cfg_type_t cfgtype;
	union {
		compat_uptr_t csiphy_params;
		compat_uptr_t csi_lane_params;
	} cfg;
};

struct sensorb_cfg_data32 {
	int cfgtype;
	
	int8_t sensor_ver;
	int8_t lens_id;
	af_value_t32 af_value;
	
	union {
		struct msm_sensor_info_t      sensor_info;
		struct msm_sensor_init_params sensor_init_params;
		compat_uptr_t                 setting;
		struct msm_sensor_i2c_sync_params sensor_i2c_sync_params;
		
		struct fuse_id32 fuse;
		
	} cfg;
	
	int cam_id;
	
	
	struct pixels_array_tt32 pixels_array;
	
};

struct msm_ois_params_t32 {
	uint16_t data_size;
	uint16_t setting_size;
	uint32_t i2c_addr;
	enum i2c_freq_mode_t i2c_freq_mode;
	enum msm_camera_i2c_reg_addr_type i2c_addr_type;
	enum msm_camera_i2c_data_type i2c_data_type;
	compat_uptr_t settings;
};

struct msm_ois_set_info_t32 {
	struct msm_ois_params_t32 ois_params;
};

struct msm_ois_cfg_data32 {
	int cfgtype;
	union {
		struct msm_ois_set_info_t32 set_info;
		compat_uptr_t settings;
	} cfg;
};

struct msm_flash_init_info_t32 {
	enum msm_flash_driver_type flash_driver_type;
	uint32_t slave_addr;
	enum i2c_freq_mode_t i2c_freq_mode;
	compat_uptr_t power_setting_array;
	compat_uptr_t settings;
};

struct msm_flash_cfg_data_t32 {
	enum msm_flash_cfg_type_t cfg_type;
	int32_t flash_current[MAX_LED_TRIGGERS];
	int32_t flash_duration[MAX_LED_TRIGGERS];
	union {
		compat_uptr_t flash_init_info;
		compat_uptr_t settings;
	} cfg;
};

#define VIDIOC_MSM_ACTUATOR_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct msm_actuator_cfg_data32)

#define VIDIOC_MSM_SENSOR_INIT_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 10, struct sensor_init_cfg_data32)

#define VIDIOC_MSM_CSIPHY_IO_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct csiphy_cfg_data32)

#define VIDIOC_MSM_SENSOR_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct sensorb_cfg_data32)

#define VIDIOC_MSM_EEPROM_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct msm_eeprom_cfg_data32)

#define VIDIOC_MSM_OIS_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 11, struct msm_ois_cfg_data32)

#define VIDIOC_MSM_CSID_IO_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 5, struct csid_cfg_data32)

#define VIDIOC_MSM_FLASH_CFG32 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 13, struct msm_flash_cfg_data_t32)
#endif

#endif


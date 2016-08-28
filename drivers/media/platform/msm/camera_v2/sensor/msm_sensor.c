/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <linux/regulator/rpm-smd-regulator.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OIS_CALIBRATION
#include "lc898123AXD_htc.h"
#endif
#ifdef CONFIG_CAM_REG_LASER
#include <linux/laser.h>
#endif

#undef CDBG
#define CDBG(fmt, args...) pr_info("[CAM]"fmt, ##args)

#ifdef CONFIG_OIS_CALIBRATION
#define OIS_COMPONENT_I2C_ADDR_WRITE 0x7C

int htc_ois_calibration(struct msm_sensor_ctrl_t *s_ctrl, int cam_id)
{
	int rc = -1;
	uint16_t cci_client_sid_backup;
    pr_info("%s:E \n", __func__);
    pr_info("%s cam_id = %d\n", __func__, cam_id);

	
	cci_client_sid_backup = s_ctrl->sensor_i2c_client->cci_client->sid;

	
	s_ctrl->sensor_i2c_client->cci_client->sid = OIS_COMPONENT_I2C_ADDR_WRITE >> 1;

    
    rc = htc_GyroReCalib(s_ctrl, cam_id);
    if (rc != 0)
          pr_err("htc_GyroReCalib fail.\n");
    else{
        rc = htc_WrGyroOffsetData();
        if (rc != 0)
            pr_err("htc_WrGyroOffsetData fail.\n");
        else
            pr_info("Gyro calibration success.\n");
    }

	
	s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup;

	return rc;
}
int htc_ois_FWupdate(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = -1;
	uint16_t cci_client_sid_backup;
    static int m_first = 0;
    static int f_first = 0;
    pr_info("%s:E s_ctrl->id = %d.\n", __func__, s_ctrl->id);

	
	cci_client_sid_backup = s_ctrl->sensor_i2c_client->cci_client->sid;

	
	s_ctrl->sensor_i2c_client->cci_client->sid = OIS_COMPONENT_I2C_ADDR_WRITE >> 1;

    if (m_first ==0||f_first==0)
    {
        rc = htc_checkFWUpdate(s_ctrl);
        if (s_ctrl->id ==0)
            m_first = 1;
        if (s_ctrl->id ==1)
            f_first = 1;
    }

	
	s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup;

	return rc;
}

#endif

static void msm_sensor_adjust_mclk(struct msm_camera_power_ctrl_t *ctrl)
{
	int idx;
	struct msm_sensor_power_setting *power_setting;
	for (idx = 0; idx < ctrl->power_setting_size; idx++) {
		power_setting = &ctrl->power_setting[idx];
		if (power_setting->seq_type == SENSOR_CLK &&
			power_setting->seq_val ==  SENSOR_CAM_MCLK) {
			if (power_setting->config_val == 24000000) {
				power_setting->config_val = 23880000;
				CDBG("%s MCLK request adjusted to 23.88MHz\n"
							, __func__);
			}
			break;
		}
	}

	return;
}

static void msm_sensor_misc_regulator(
	struct msm_sensor_ctrl_t *sctrl, uint32_t enable)
{
	int32_t rc = 0;
	if (enable) {
		sctrl->misc_regulator = (void *)rpm_regulator_get(
			&sctrl->pdev->dev, sctrl->sensordata->misc_regulator);
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(sctrl->misc_regulator,
				RPM_REGULATOR_MODE_HPM);
			if (rc < 0) {
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
				rpm_regulator_put(sctrl->misc_regulator);
			}
		} else {
			pr_err("%s: Failed to vote for rpm regulator on %s: %d\n",
				__func__,
				sctrl->sensordata->misc_regulator, rc);
		}
	} else {
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(
				(struct rpm_regulator *)sctrl->misc_regulator,
				RPM_REGULATOR_MODE_AUTO);
			if (rc < 0)
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
			rpm_regulator_put(sctrl->misc_regulator);
		}
	}
}

int32_t msm_sensor_free_sensor_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	if (!s_ctrl->pdev && !s_ctrl->sensor_i2c_client->client)
		return 0;
	kfree(s_ctrl->sensordata->slave_info);
	kfree(s_ctrl->sensordata->cam_slave_info);
	kfree(s_ctrl->sensordata->actuator_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
	kfree(s_ctrl->sensordata->power_info.power_setting);
	kfree(s_ctrl->sensordata->power_info.power_down_setting);
	kfree(s_ctrl->sensordata->csi_lane_params);
	kfree(s_ctrl->sensordata->sensor_info);
	kfree(s_ctrl->sensordata->power_info.clk_info);
	kfree(s_ctrl->sensordata);
	return 0;
}

static struct msm_cam_clk_info cam_8974_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 24000000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};

int msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_power_ctrl_t *power_info;
	enum msm_camera_device_type_t sensor_device_type;
	struct msm_camera_i2c_client *sensor_i2c_client;

	if (!s_ctrl) {
		pr_err("%s:%d failed: s_ctrl %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	if (s_ctrl->is_csid_tg_mode)
		return 0;

	power_info = &s_ctrl->sensordata->power_info;
	sensor_device_type = s_ctrl->sensor_device_type;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;

	if (!power_info || !sensor_i2c_client) {
		pr_err("%s:%d failed: power_info %p sensor_i2c_client %p\n",
			__func__, __LINE__, power_info, sensor_i2c_client);
		return -EINVAL;
	}
	return msm_camera_power_down(power_info, sensor_device_type,
		sensor_i2c_client);
}

int msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	uint32_t retry = 0;

#ifdef CONFIG_CAM_REG_LASER
	static int first = 1;
#endif

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	if (s_ctrl->is_csid_tg_mode)
		return 0;

	power_info = &s_ctrl->sensordata->power_info;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!power_info || !sensor_i2c_client || !slave_info ||
		!sensor_name) {
		pr_err("%s:%d failed: %p %p %p %p\n",
			__func__, __LINE__, power_info,
			sensor_i2c_client, slave_info, sensor_name);
		return -EINVAL;
	}

#ifdef CONFIG_CAM_REG_LASER
	if (first)
	{
		CDBG("%s: Call Laser_poweron_by_camera()", __func__);
		Laser_poweron_by_camera();
		CDBG("%s: Call Laser_poweroff_by_camera()", __func__);
		Laser_poweroff_by_camera();
		first = 0;
		msleep(1);
	}
#endif

	if (s_ctrl->set_mclk_23880000)
		msm_sensor_adjust_mclk(power_info);

	for (retry = 0; retry < 3; retry++) {
		rc = msm_camera_power_up(power_info, s_ctrl->sensor_device_type,
			sensor_i2c_client);
		if (rc < 0)
			return rc;
		rc = msm_sensor_check_id(s_ctrl);
		if (rc < 0) {
			msm_camera_power_down(power_info,
				s_ctrl->sensor_device_type, sensor_i2c_client);
			msleep(20);
			continue;
		} else {
			break;
		}
	}
    htc_ois_FWupdate(s_ctrl);
	return rc;
}

static uint16_t msm_sensor_id_by_mask(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t chipid)
{
	uint16_t sensor_id = chipid;
	int16_t sensor_id_mask = s_ctrl->sensordata->slave_info->sensor_id_mask;

	if (!sensor_id_mask)
		sensor_id_mask = ~sensor_id_mask;

	sensor_id &= sensor_id_mask;
	sensor_id_mask &= -sensor_id_mask;
	sensor_id_mask -= 1;
	while (sensor_id_mask) {
		sensor_id_mask >>= 1;
		sensor_id >>= 1;
	}
	return sensor_id;
}

#define EEPROM_COMPONENT_I2C_ADDR_WRITE 0xA0
int msm_sensor_read_fuseid(struct sensorb_cfg_data *cdata, struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t address = 0;
	uint16_t read_data = 0;
	int rc = 0;
	int i = 0;
	int j = 0;
	uint16_t cci_client_sid_backup;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	static uint8_t main_otp[20];
	static uint8_t defect_pixel_of_main_otp[160]; 
	static struct pixels_array_tt pixels_array = { { { 0, 0 } }, 0 }; 
	bool check_defect_pixel_count; 
	int index = 0;
	
	int index_for_dp = 0;
	int pix_count = 0;
	int y = 0;
	int y_t = 0;
	int x = 0;
	int x_t = 0;
	int delta_x = 0;
	int delta_y = 0;
	
	const short id_addr[10] = {0x000D,0x000E,0x000F,0x0010,0x0011,0x0012,0x0013,0x0014,0x0015,0x0016};
	static uint16_t front_otp_data[10] = {0,0,0,0,0,0,0,0,0,0}; 
	static uint16_t front_id_data[4] = {0,0,0,0}; 
	static int32_t valid_page=-1;
	static int32_t SN_valid_page=-1;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	CDBG("%s: +", __func__);
	if (!sensor_i2c_client || !slave_info )
	{
		CDBG("%s: !sensor_i2c_client || !slave_info  return", __func__);
		return 0;
	}
	cci_client_sid_backup = slave_info->sensor_slave_addr;
	if(strncmp("imx377_htc", s_ctrl->sensordata->sensor_name, sizeof("imx377_htc")) == 0)
	{
		static int first= true;
		if(first == true)
		{
		    s_ctrl->sensor_i2c_client->cci_client->sid = EEPROM_COMPONENT_I2C_ADDR_WRITE >> 1;
		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
		    for(address = 0; address < 0xb ; address++)
		    {
		        read_data = 0;
		        msleep(1);
		        rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		        main_otp[index] = read_data & 0xff;
		        CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		        index++;
		    }
		    
		    for(address = 0x15; address <= 0xB4 ; address += 0x04 )
		    {
			    check_defect_pixel_count = false;
			    for(i = 0x0; i < 0x04; i++ )
			    {
				    read_data = 0;
				    msleep(1);
				    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address+i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
				    defect_pixel_of_main_otp[index_for_dp] = read_data & 0xff;
				    CDBG("%s: read(0x%x, 0x%x), defect_pixel_of_main_otp[%d] = 0x%x", __func__, address+i, read_data, index_for_dp, defect_pixel_of_main_otp[index_for_dp]);
				    if(defect_pixel_of_main_otp[index_for_dp] != 0xff)
				    {
				    	check_defect_pixel_count = true;
				    }
				    index_for_dp++;
			    }
			    if(check_defect_pixel_count)
			    {
				    CDBG("%s: defect_pixels_index : %d", __func__, pix_count);
		
				    y = 0;
				    y_t = 0;
				    y = defect_pixel_of_main_otp[index_for_dp-4];
				    y = y << 5;
				    y_t = defect_pixel_of_main_otp[index_for_dp-3] >> 3;
				    pixels_array.pix[pix_count].y = y | y_t;
		
				    x = 0;
				    x_t = 0;
				    x = defect_pixel_of_main_otp[index_for_dp-3];
				    x = x & 0x07;
				    x = x << 8;
				    x_t = (x | defect_pixel_of_main_otp[index_for_dp-2]) << 2;
				    pixels_array.pix[pix_count].x =  x_t | (defect_pixel_of_main_otp[index_for_dp-1] >> 6 );
		
				    pixels_array.pix[pix_count].y -= 18; 
				    pixels_array.pix[pix_count++].x -= 26; 
				    CDBG("%s: pixel_x = (0x%x,%d), pixel_y = (0x%x,%d)", __func__, pixels_array.pix[pix_count-1].x, pixels_array.pix[pix_count-1].x, pixels_array.pix[pix_count-1].y, pixels_array.pix[pix_count-1].y);
		
				    delta_y = 0;
				    delta_x = 0;
				    delta_y = (defect_pixel_of_main_otp[index_for_dp-1] & 0x38) >> 3;
				    delta_x = (defect_pixel_of_main_otp[index_for_dp-1] & 0x07);
				    CDBG("%s: delta_y = (0x%x,%d)", __func__, delta_y, delta_y);
				    CDBG("%s: delta_x = (0x%x,%d)", __func__, delta_x, delta_x);
		
				    if( delta_y != 0 || delta_x != 0)
				    {
				    	CDBG("%s: defect_pixels_index : %d", __func__, pix_count);
					    if( (delta_y & 0x04) > 1 )
					    {
					    	pixels_array.pix[pix_count].x = pixels_array.pix[pix_count-1].x + delta_x;
					    	pixels_array.pix[pix_count].y = pixels_array.pix[pix_count-1].y - (delta_y & 0x03);
					    	CDBG("%s: pixel_x = (0x%x,%d), pixel_y = (0x%x,%d)", __func__, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].y, pixels_array.pix[pix_count].y);
					    }
					    else
					    {
					    	pixels_array.pix[pix_count].x = pixels_array.pix[pix_count-1].x + delta_x;
					    	pixels_array.pix[pix_count].y = pixels_array.pix[pix_count-1].y + (delta_y & 0x03);
					    	CDBG("%s: pixel_x = (0x%x,%d), pixel_y = (0x%x,%d)", __func__, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].y, pixels_array.pix[pix_count].y);
					    }
					    pix_count++;
				    }
			    }
			    else
			    {
			    	break;
			    }
		    }
		    pixels_array.count = pix_count;
		    CDBG("%s: Total_DP_Count = %d", __func__, pixels_array.count);
		    
		    s_ctrl->sensor_i2c_client->cci_client->sid = 0x56;
		    msleep(1);
		    read_data = 0;
		    address =  0x56;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    msleep(1);
		    read_data = 0;
		    address =  0x57;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    msleep(1);
		    read_data = 0;
		    address =  0x58;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    msleep(1);
		    read_data = 0;
		    address =  0x59;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		    s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup >> 1;
		    first= false;
		}
		
		if (cdata != NULL) {
		cdata->cfg.fuse.fuse_id_word1 = main_otp[11];
		cdata->cfg.fuse.fuse_id_word2 = main_otp[12];
		cdata->cfg.fuse.fuse_id_word3 = main_otp[13];
		cdata->cfg.fuse.fuse_id_word4 = main_otp[14];

		
		cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word2;
		cdata->af_value.VCM_VENDOR_ID_VERSION = main_otp[4];
		cdata->af_value.AF_INF_MSB = main_otp[5];
		cdata->af_value.AF_INF_LSB = main_otp[6];
		cdata->af_value.AF_MACRO_MSB = main_otp[9];
		cdata->af_value.AF_MACRO_LSB = main_otp[10];

		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);

		pr_info("PMEif: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
		  cdata->cfg.fuse.fuse_id_word1,
		  cdata->cfg.fuse.fuse_id_word2,
		  cdata->cfg.fuse.fuse_id_word3,
		  cdata->cfg.fuse.fuse_id_word4);

		pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
		pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
		pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
		pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);

		cdata->af_value.VCM_VENDOR = main_otp[0];
		
		
		cdata->pixels_array.count = pixels_array.count;
		for(i = 0 ; i < cdata->pixels_array.count ; i++)
		{
			cdata->pixels_array.pix[i].x = pixels_array.pix[i].x;
			cdata->pixels_array.pix[i].y = pixels_array.pix[i].y;
		}
		

		strlcpy(cdata->af_value.ACT_NAME, "lc898214_act", sizeof("lc898214_act"));
		pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
		}
		else {
		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);
		}
	}
    if(strncmp("s5k4e6_htc", s_ctrl->sensordata->sensor_name, sizeof("s5k4e6_htc")) == 0)
    {
		static int front_first= true;
		if(front_first == true)
        {
            front_first= false;
            
            for (i = 53 ; i >= 51 ; i--)
            {
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0xFCFC, 0x4000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                msleep(10);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6028, 0x2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0x000B, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, i, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0X0009, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, 0X01, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A00, 0X0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                msleep(5);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602C, 0X2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                for (j = 0 ; j <4; j++)
                {
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602E, id_addr[j], MSM_CAMERA_I2C_WORD_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_write failed\n", __func__);
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x6F12, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_read failed\n", __func__);

                    front_id_data[j] = read_data & 0xff;

                    if (read_data)
                        SN_valid_page = i;
                }

                if (SN_valid_page!=-1)
                    break;

            }
            
            for (i = 50 ; i >= 48 ; i--)
            {
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0xFCFC, 0x4000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                msleep(10);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6028, 0x2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0x000B, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, i, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0X0009, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, 0X01, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A00, 0X0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                msleep(5);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602C, 0X2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                for (j = 0 ; j <10; j++)
                {
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602E, id_addr[j], MSM_CAMERA_I2C_WORD_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_write failed\n", __func__);
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x6F12, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_read failed\n", __func__);

                    front_otp_data[j] = read_data & 0xff;

                    if (read_data)
                        valid_page = i;

                }
                if (valid_page!=-1)
                    break;
            }
        }
        if (cdata != NULL) {
            cdata->cfg.fuse.fuse_id_word1 = front_id_data[0];
            cdata->cfg.fuse.fuse_id_word2 = front_id_data[1];
            cdata->cfg.fuse.fuse_id_word3 = front_id_data[2];
            cdata->cfg.fuse.fuse_id_word4 = front_id_data[3];
            pr_info("s5k4e6_htc: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
                    cdata->cfg.fuse.fuse_id_word1,
                    cdata->cfg.fuse.fuse_id_word2,
                    cdata->cfg.fuse.fuse_id_word3,
                    cdata->cfg.fuse.fuse_id_word4);
            cdata->lens_id = front_otp_data[1];
            
            cdata->af_value.VCM_VENDOR = front_otp_data[0];
            cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word1;
            cdata->af_value.VCM_VENDOR_ID_VERSION = front_otp_data[4];
            cdata->af_value.AF_INF_MSB = front_otp_data [5];
            cdata->af_value.AF_INF_LSB = front_otp_data [6];
            cdata->af_value.AF_MACRO_MSB = front_otp_data [7];
            cdata->af_value.AF_MACRO_LSB = front_otp_data [8];
            strlcpy(cdata->af_value.ACT_NAME, "lc898123", sizeof("lc898123"));
            pr_info("%s: s5k4e6_htcOTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);

            pr_info("%s: s5k4e6_htc OTP Module vendor = 0x%x\n",               __func__,  front_otp_data[0]);
            pr_info("%s: s5k4e6_htc OTP LENS = 0x%x\n",                        __func__,  front_otp_data[1]);
            pr_info("%s: s5k4e6_htc OTP Sensor Version = 0x%x\n",              __func__,  front_otp_data[2]);
            pr_info("%s: s5k4e6_htc OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  front_otp_data[3]);
            pr_info("%s: s5k4e6_htc OTP Actuator vender ID & Version = 0x%x\n",__func__,  front_otp_data[4]);
            pr_info("%s: s5k4e6_htc OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
            pr_info("%s: s5k4e6_htc OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
            pr_info("%s: s5k4e6_htc OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
            pr_info("%s: s5k4e6_htc OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);
        }
        else {
            pr_info("%s: s5k4e6_htc OTP Module vendor = 0x%x\n",               __func__,  front_otp_data[0]);
            pr_info("%s: s5k4e6_htc OTP LENS = 0x%x\n",                        __func__,  front_otp_data[1]);
            pr_info("%s: s5k4e6_htc OTP Sensor Version = 0x%x\n",              __func__,  front_otp_data[2]);
            pr_info("%s: s5k4e6_htc OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  front_otp_data[3]);
            pr_info("%s: s5k4e6_htc OTP Actuator vender ID & Version = 0x%x\n",__func__,  front_otp_data[4]);
        }
        return rc;

    }
	
	if(strncmp("ov12890_htc", s_ctrl->sensordata->sensor_name, sizeof("ov12890_htc")) == 0)
	{
		static int first= true;
		if(first == true)
		{
		    s_ctrl->sensor_i2c_client->cci_client->sid = EEPROM_COMPONENT_I2C_ADDR_WRITE >> 1;
		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
		    for(address = 0; address < 0xb ; address++)
		    {
		        read_data = 0;
		        msleep(1);
		        rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		        main_otp[index] = read_data & 0xff;
		        CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		        index++;
		    }
		    s_ctrl->sensor_i2c_client->cci_client->sid = 0x56;
		    msleep(1);
		    read_data = 0;
		    address =  0x56;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    msleep(1);
		    read_data = 0;
		    address =  0x57;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    msleep(1);
		    read_data = 0;
		    address =  0x58;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    msleep(1);
		    read_data = 0;
		    address =  0x59;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		    s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup >> 1;
		    first= false;
		}

		if (cdata != NULL) {
		cdata->cfg.fuse.fuse_id_word1 = main_otp[11];
		cdata->cfg.fuse.fuse_id_word2 = main_otp[12];
		cdata->cfg.fuse.fuse_id_word3 = main_otp[13];
		cdata->cfg.fuse.fuse_id_word4 = main_otp[14];

		
		cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word2;
		cdata->af_value.VCM_VENDOR_ID_VERSION = main_otp[4];
		cdata->af_value.AF_INF_MSB = main_otp[5];
		cdata->af_value.AF_INF_LSB = main_otp[6];
		cdata->af_value.AF_MACRO_MSB = main_otp[9];
		cdata->af_value.AF_MACRO_LSB = main_otp[10];

		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);

		pr_info("PMEos: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
		  cdata->cfg.fuse.fuse_id_word1,
		  cdata->cfg.fuse.fuse_id_word2,
		  cdata->cfg.fuse.fuse_id_word3,
		  cdata->cfg.fuse.fuse_id_word4);

		pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
		pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
		pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
		pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);

		cdata->af_value.VCM_VENDOR = main_otp[0];

		strlcpy(cdata->af_value.ACT_NAME, "lc898214_act", sizeof("lc898214_act"));
		pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
		}
		else {
		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);
		}
	}
	

	CDBG("%s: -", __func__);
	return 0;
}
#ifdef CONFIG_COMPAT
int msm_sensor_read_fuseid32(struct sensorb_cfg_data32 *cdata, struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t address = 0;
	uint16_t read_data = 0;
	int rc = 0;
	int i = 0;
	int j = 0;
	uint16_t cci_client_sid_backup;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	static uint8_t main_otp[20];
	static uint8_t defect_pixel_of_main_otp[160]; 
	static struct pixels_array_tt pixels_array = { { { 0, 0 } }, 0 }; 
	bool check_defect_pixel_count; 
	const short id_addr[10] = {0x000D,0x000E,0x000F,0x0010,0x0011,0x0012,0x0013,0x0014,0x0015,0x0016};
	static uint16_t front_otp_data[10] = {0,0,0,0,0,0,0,0,0,0}; 
	static uint16_t front_id_data[4] = {0,0,0,0}; 
	static int32_t valid_page=-1;
	static int32_t SN_valid_page=-1;
	int index = 0;
	
	int index_for_dp = 0;
	int pix_count = 0;
	int y = 0;
	int y_t = 0;
	int x = 0;
	int x_t = 0;
	int delta_x = 0;
	int delta_y = 0;
	
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	CDBG("%s: +", __func__);
	if (!sensor_i2c_client || !slave_info )
	{
		CDBG("%s: !sensor_i2c_client || !slave_info  return", __func__);
		return 0;
	}
	cci_client_sid_backup = slave_info->sensor_slave_addr;
	if(strncmp("imx377_htc", s_ctrl->sensordata->sensor_name, sizeof("imx377_htc")) == 0)
	{
		static int first= true;
		if(first == true)
		{
		    s_ctrl->sensor_i2c_client->cci_client->sid = EEPROM_COMPONENT_I2C_ADDR_WRITE >> 1;
		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
		    for(address = 0; address < 0xb ; address++)
		    {
		        read_data = 0;
		        msleep(1);
		        rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		        main_otp[index] = read_data & 0xff;
		        CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		        index++;
		    }
		    
		    for(address = 0x15; address <= 0xB4 ; address += 0x04 )
		    {
			    check_defect_pixel_count = false;
			    for(i = 0x0; i < 0x04; i++ )
			    {
				    read_data = 0;
				    msleep(1);
				    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address+i, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
				    defect_pixel_of_main_otp[index_for_dp] = read_data & 0xff;
				    CDBG("%s: read(0x%x, 0x%x), defect_pixel_of_main_otp[%d] = 0x%x", __func__, address+i, read_data, index_for_dp, defect_pixel_of_main_otp[index_for_dp]);
				    if(defect_pixel_of_main_otp[index_for_dp] != 0xff)
				    {
				    	check_defect_pixel_count = true;
				    }
				    index_for_dp++;
			    }
			    if(check_defect_pixel_count)
			    {
				    CDBG("%s: defect_pixels_index : %d", __func__, pix_count);
		
				    y = 0;
				    y_t = 0;
				    y = defect_pixel_of_main_otp[index_for_dp-4];
				    y = y << 5;
				    y_t = defect_pixel_of_main_otp[index_for_dp-3] >> 3;
				    pixels_array.pix[pix_count].y = y | y_t;
		
				    x = 0;
				    x_t = 0;
				    x = defect_pixel_of_main_otp[index_for_dp-3];
				    x = x & 0x07;
				    x = x << 8;
				    x_t = (x | defect_pixel_of_main_otp[index_for_dp-2]) << 2;
				    pixels_array.pix[pix_count].x =  x_t | (defect_pixel_of_main_otp[index_for_dp-1] >> 6 );
		
				    pixels_array.pix[pix_count].y -= 18; 
				    pixels_array.pix[pix_count++].x -= 26; 
				    CDBG("%s: pixel_x = (0x%x,%d), pixel_y = (0x%x,%d)", __func__, pixels_array.pix[pix_count-1].x, pixels_array.pix[pix_count-1].x, pixels_array.pix[pix_count-1].y, pixels_array.pix[pix_count-1].y);
		
				    delta_y = 0;
				    delta_x = 0;
				    delta_y = (defect_pixel_of_main_otp[index_for_dp-1] & 0x38) >> 3;
				    delta_x = (defect_pixel_of_main_otp[index_for_dp-1] & 0x07);
				    CDBG("%s: delta_y = (0x%x,%d)", __func__, delta_y, delta_y);
				    CDBG("%s: delta_x = (0x%x,%d)", __func__, delta_x, delta_x);
		
				    if( delta_y != 0 || delta_x != 0)
				    {
				    	CDBG("%s: defect_pixels_index : %d", __func__, pix_count);
					    if( (delta_y & 0x04) > 1 )
					    {
					    	pixels_array.pix[pix_count].x = pixels_array.pix[pix_count-1].x + delta_x;
					    	pixels_array.pix[pix_count].y = pixels_array.pix[pix_count-1].y - (delta_y & 0x03);
					    	CDBG("%s: pixel_x = (0x%x,%d), pixel_y = (0x%x,%d)", __func__, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].y, pixels_array.pix[pix_count].y);
					    }
					    else
					    {
					    	pixels_array.pix[pix_count].x = pixels_array.pix[pix_count-1].x + delta_x;
					    	pixels_array.pix[pix_count].y = pixels_array.pix[pix_count-1].y + (delta_y & 0x03);
					    	CDBG("%s: pixel_x = (0x%x,%d), pixel_y = (0x%x,%d)", __func__, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].x, pixels_array.pix[pix_count].y, pixels_array.pix[pix_count].y);
					    }
					    pix_count++;
				    }
			    }
			    else
			    {
			    	break;
			    }
		    }
		    pixels_array.count = pix_count;
		    CDBG("%s: Total_DP_Count = %d", __func__, pixels_array.count);
		    
		    s_ctrl->sensor_i2c_client->cci_client->sid = 0x56;
		    msleep(1);
		    read_data = 0;
		    address =  0x56;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    msleep(1);
		    read_data = 0;
		    address =  0x57;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    msleep(1);
		    read_data = 0;
		    address =  0x58;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    msleep(1);
		    read_data = 0;
		    address =  0x59;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;
	    
		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		    s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup >> 1;
		    first= false;
		}
		
		if (cdata != NULL) {
		cdata->cfg.fuse.fuse_id_word1 = main_otp[11];
		cdata->cfg.fuse.fuse_id_word2 = main_otp[12];
		cdata->cfg.fuse.fuse_id_word3 = main_otp[13];
		cdata->cfg.fuse.fuse_id_word4 = main_otp[14];

		
		cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word2;
		cdata->af_value.VCM_VENDOR_ID_VERSION = main_otp[4];
		cdata->af_value.AF_INF_MSB = main_otp[5];
		cdata->af_value.AF_INF_LSB = main_otp[6];
		cdata->af_value.AF_MACRO_MSB = main_otp[9];
		cdata->af_value.AF_MACRO_LSB = main_otp[10];

		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);

		pr_info("PMEif: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
		  cdata->cfg.fuse.fuse_id_word1,
		  cdata->cfg.fuse.fuse_id_word2,
		  cdata->cfg.fuse.fuse_id_word3,
		  cdata->cfg.fuse.fuse_id_word4);

		pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
		pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
		pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
		pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);

		cdata->af_value.VCM_VENDOR = main_otp[0];

		
		cdata->pixels_array.count = pixels_array.count;
		for(i = 0 ; i < cdata->pixels_array.count ; i++)
		{
			cdata->pixels_array.pix[i].x = pixels_array.pix[i].x;
			cdata->pixels_array.pix[i].y = pixels_array.pix[i].y;
		}
		

		strlcpy(cdata->af_value.ACT_NAME, "lc898214_act", sizeof("lc898214_act"));
		pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
		}
		else {
		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);
		}
	}
    if(strncmp("s5k4e6_htc", s_ctrl->sensordata->sensor_name, sizeof("s5k4e6_htc")) == 0)
    {
		static int front_first= true;
		if(front_first == true)
        {
            front_first= false;
            for (i = 53 ; i >= 51 ; i--)
            {
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0xFCFC, 0x4000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                msleep(10);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6028, 0x2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0x000B, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, i, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0X0009, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, 0X01, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A00, 0X0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                msleep(5);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602C, 0X2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                for (j = 0 ; j <4; j++)
                {
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602E, id_addr[j], MSM_CAMERA_I2C_WORD_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_write failed\n", __func__);
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x6F12, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_read failed\n", __func__);

                    front_id_data[j] = read_data & 0xff;

                    if (read_data)
                        SN_valid_page = i;

                }
                if (SN_valid_page!=-1)
                    break;

            }
            
            for (i = 50 ; i >= 48 ; i--)
            {
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0xFCFC, 0x4000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0100, 0x0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                msleep(10);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6028, 0x2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0x000B, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, i, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602A, 0X0009, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6F12, 0X01, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0A00, 0X0100, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                msleep(5);
                
                rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602C, 0X2000, MSM_CAMERA_I2C_WORD_DATA);
                if (rc < 0)
					pr_err("%s: i2c_write failed\n", __func__);

                
                for (j = 0 ; j <10; j++)
                {
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x602E, id_addr[j], MSM_CAMERA_I2C_WORD_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_write failed\n", __func__);
                    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x6F12, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
                    if (rc < 0)
                        pr_err("%s: i2c_read failed\n", __func__);

                    front_otp_data[j] = read_data & 0xff;

                    if (read_data)
                        valid_page = i;

                }
                if (valid_page!=-1)
                    break;
            }
        }
        if (cdata != NULL) {
            cdata->cfg.fuse.fuse_id_word1 = front_id_data[0];
            cdata->cfg.fuse.fuse_id_word2 = front_id_data[1];
            cdata->cfg.fuse.fuse_id_word3 = front_id_data[2];
            cdata->cfg.fuse.fuse_id_word4 = front_id_data[3];
            pr_info("s5k4e6_htc: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
                    cdata->cfg.fuse.fuse_id_word1,
                    cdata->cfg.fuse.fuse_id_word2,
                    cdata->cfg.fuse.fuse_id_word3,
                    cdata->cfg.fuse.fuse_id_word4);
            cdata->lens_id = front_otp_data[1];
            
            cdata->af_value.VCM_VENDOR = front_otp_data[0];
            cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word1;
            cdata->af_value.VCM_VENDOR_ID_VERSION = front_otp_data[4];
            cdata->af_value.AF_INF_MSB = front_otp_data [5];
            cdata->af_value.AF_INF_LSB = front_otp_data [6];
            cdata->af_value.AF_MACRO_MSB = front_otp_data [7];
            cdata->af_value.AF_MACRO_LSB = front_otp_data [8];
            strlcpy(cdata->af_value.ACT_NAME, "lc898123", sizeof("lc898123"));
            pr_info("%s: s5k4e6_htcOTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);

            pr_info("%s: s5k4e6_htc OTP Module vendor = 0x%x\n",               __func__,  front_otp_data[0]);
            pr_info("%s: s5k4e6_htc OTP LENS = 0x%x\n",                        __func__,  front_otp_data[1]);
            pr_info("%s: s5k4e6_htc OTP Sensor Version = 0x%x\n",              __func__,  front_otp_data[2]);
            pr_info("%s: s5k4e6_htc OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  front_otp_data[3]);
            pr_info("%s: s5k4e6_htc OTP Actuator vender ID & Version = 0x%x\n",__func__,  front_otp_data[4]);
            pr_info("%s: s5k4e6_htc OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
            pr_info("%s: s5k4e6_htc OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
            pr_info("%s: s5k4e6_htc OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
            pr_info("%s: s5k4e6_htc OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);
        }
        else {
            pr_info("%s: s5k4e6_htc OTP Module vendor = 0x%x\n",               __func__,  front_otp_data[0]);
            pr_info("%s: s5k4e6_htc OTP LENS = 0x%x\n",                        __func__,  front_otp_data[1]);
            pr_info("%s: s5k4e6_htc OTP Sensor Version = 0x%x\n",              __func__,  front_otp_data[2]);
            pr_info("%s: s5k4e6_htc OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  front_otp_data[3]);
            pr_info("%s: s5k4e6_htc OTP Actuator vender ID & Version = 0x%x\n",__func__,  front_otp_data[4]);
        }
        return rc;

    }
	
	if(strncmp("ov12890_htc", s_ctrl->sensordata->sensor_name, sizeof("ov12890_htc")) == 0)
	{
		static int first= true;
		if(first == true)
		{
		    s_ctrl->sensor_i2c_client->cci_client->sid = EEPROM_COMPONENT_I2C_ADDR_WRITE >> 1;
		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
		    for(address = 0; address < 0xb ; address++)
		    {
		        read_data = 0;
		        msleep(1);
		        rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		        main_otp[index] = read_data & 0xff;
		        CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		        index++;
		    }
		    s_ctrl->sensor_i2c_client->cci_client->sid = 0x56;
		    msleep(1);
		    read_data = 0;
		    address =  0x56;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    msleep(1);
		    read_data = 0;
		    address =  0x57;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    msleep(1);
		    read_data = 0;
		    address =  0x58;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    msleep(1);
		    read_data = 0;
		    address =  0x59;
		    rc = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, address, &read_data, MSM_CAMERA_I2C_BYTE_DATA);
		    main_otp[index] = read_data & 0xff;
		    CDBG("%s: read(0x%x, 0x%x), main_otp[%d] = 0x%x", __func__, address, read_data, index, main_otp[index]);
		    index++;

		    sensor_i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		    s_ctrl->sensor_i2c_client->cci_client->sid = cci_client_sid_backup >> 1;
		    first= false;
		}

		if (cdata != NULL) {
		cdata->cfg.fuse.fuse_id_word1 = main_otp[11];
		cdata->cfg.fuse.fuse_id_word2 = main_otp[12];
		cdata->cfg.fuse.fuse_id_word3 = main_otp[13];
		cdata->cfg.fuse.fuse_id_word4 = main_otp[14];

		
		cdata->af_value.MODULE_ID_AB = cdata->cfg.fuse.fuse_id_word2;
		cdata->af_value.VCM_VENDOR_ID_VERSION = main_otp[4];
		cdata->af_value.AF_INF_MSB = main_otp[5];
		cdata->af_value.AF_INF_LSB = main_otp[6];
		cdata->af_value.AF_MACRO_MSB = main_otp[9];
		cdata->af_value.AF_MACRO_LSB = main_otp[10];

		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);

		pr_info("PMEos: fuse->fuse_id : 0x%x 0x%x 0x%x 0x%x\n",
		  cdata->cfg.fuse.fuse_id_word1,
		  cdata->cfg.fuse.fuse_id_word2,
		  cdata->cfg.fuse.fuse_id_word3,
		  cdata->cfg.fuse.fuse_id_word4);

		pr_info("%s: OTP Infinity position code (MSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_MSB);
		pr_info("%s: OTP Infinity position code (LSByte) = 0x%x\n", __func__,  cdata->af_value.AF_INF_LSB);
		pr_info("%s: OTP Macro position code (MSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_MSB);
		pr_info("%s: OTP Macro position code (LSByte) = 0x%x\n",    __func__,  cdata->af_value.AF_MACRO_LSB);

		cdata->af_value.VCM_VENDOR = main_otp[0];

		strlcpy(cdata->af_value.ACT_NAME, "lc898214_act", sizeof("lc898214_act"));
		pr_info("%s: OTP Actuator Name = %s\n",__func__, cdata->af_value.ACT_NAME);
		}
		else {
		pr_info("%s: OTP Module vendor = 0x%x\n",               __func__,  main_otp[0]);
		pr_info("%s: OTP LENS = 0x%x\n",                        __func__,  main_otp[1]);
		pr_info("%s: OTP Sensor Version = 0x%x\n",              __func__,  main_otp[2]);
		pr_info("%s: OTP Driver IC Vendor & Version = 0x%x\n",  __func__,  main_otp[3]);
		pr_info("%s: OTP Actuator vender ID & Version = 0x%x\n",__func__,  main_otp[4]);
		}
	}
	

	CDBG("%s: -", __func__);
	return 0;
}
#endif
int msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
    
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n",
			__func__, __LINE__, sensor_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		
		if(strncmp("imx377_htc", sensor_name, sizeof("imx377_htc")) == 0)
		{
			pr_err("%s: PMEif_htc: read id failed\n", __func__);
		}
		else if(strncmp("ov12890_htc", sensor_name, sizeof("ov12890_htc")) == 0)
		{
			pr_err("%s: PMEos_htc: read id failed\n", __func__);
		}
		else
		
		pr_err("%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}

	CDBG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		slave_info->sensor_id);
	if (msm_sensor_id_by_mask(s_ctrl, chipid) != slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static struct msm_sensor_ctrl_t *get_sctrl(struct v4l2_subdev *sd)
{
	return container_of(container_of(sd, struct msm_sd_subdev, sd),
		struct msm_sensor_ctrl_t, msm_sd);
}

static void msm_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &s_ctrl->stop_setting);
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;

		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			pr_err("s_ctrl->func_tbl NULL\n");
		}
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return;
}

static int msm_sensor_get_af_status(struct msm_sensor_ctrl_t *s_ctrl,
			void __user *argp)
{
	return 0;
}

static long msm_sensor_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	void __user *argp = (void __user *)arg;
	if (!s_ctrl) {
		pr_err("%s s_ctrl NULL\n", __func__);
		return -EBADF;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG:
#ifdef CONFIG_COMPAT
		if (is_compat_task())
			rc = s_ctrl->func_tbl->sensor_config32(s_ctrl, argp);
		else
#endif
			rc = s_ctrl->func_tbl->sensor_config(s_ctrl, argp);
		return rc;
	case VIDIOC_MSM_SENSOR_GET_AF_STATUS:
		return msm_sensor_get_af_status(s_ctrl, argp);
	case VIDIOC_MSM_SENSOR_RELEASE:
	case MSM_SD_SHUTDOWN:
		msm_sensor_stop_stream(s_ctrl);
		return 0;
	case MSM_SD_NOTIFY_FREEZE:
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long msm_sensor_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG32:
		cmd = VIDIOC_MSM_SENSOR_CFG;
	default:
		return msm_sensor_subdev_ioctl(sd, cmd, arg);
	}
}

long msm_sensor_subdev_fops_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_sensor_subdev_do_ioctl);
}
#if 1
int msm_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
#else
static int msm_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
#endif
{
	struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;
	int32_t rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	
	if(strncmp("imx377_htc", s_ctrl->sensordata->sensor_name, sizeof("imx377_htc")) == 0)
	{
		CDBG("%s:%d PMEif_htc cfgtype = %d\n", __func__, __LINE__, cdata->cfgtype);
	}
	else if(strncmp("ov12890_htc", s_ctrl->sensordata->sensor_name, sizeof("ov12890_htc")) == 0)
	{
		CDBG("%s:%d PMEos_htc cfgtype = %d\n", __func__, __LINE__, cdata->cfgtype);
	}
	else
	
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
			cdata->cfg.sensor_info.subdev_intf[i] =
				s_ctrl->sensordata->sensor_info->subdev_intf[i];
		}
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
			CDBG("%s:%d subdev_intf[%d] %d\n", __func__, __LINE__,
				i, cdata->cfg.sensor_info.subdev_intf[i]);
		}
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_WRITE_I2C_ARRAY:
	case CFG_WRITE_I2C_ARRAY_SYNC:
	case CFG_WRITE_I2C_ARRAY_SYNC_BLOCK:
	case CFG_WRITE_I2C_ARRAY_ASYNC: {
		struct msm_camera_i2c_reg_setting32 conf_array32;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.data_type = conf_array32.data_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);

		if (!conf_array.size ||
			conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
			(void *)(conf_array.reg_setting),
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;

		if (CFG_WRITE_I2C_ARRAY == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table(s_ctrl->sensor_i2c_client,
				&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_ASYNC == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_async(s_ctrl->sensor_i2c_client,
				&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_SYNC_BLOCK == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync_block(
				s_ctrl->sensor_i2c_client,
				&conf_array);
		else
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync(s_ctrl->sensor_i2c_client,
				&conf_array);

		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		struct msm_camera_i2c_read_config *read_config_ptr = NULL;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		read_config_ptr =
			(struct msm_camera_i2c_read_config *)
			compat_ptr(cdata->cfg.setting);

		if (copy_from_user(&read_config, read_config_ptr,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		}
		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		read_config_ptr->data = local_data;
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting32 conf_array32;
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_seq_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	case CFG_POWER_DOWN:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting32 stop_setting32;
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (copy_from_user(&stop_setting32,
				(void *)compat_ptr((cdata->cfg.setting)),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		stop_setting->addr_type = stop_setting32.addr_type;
		stop_setting->data_type = stop_setting32.data_type;
		stop_setting->delay = stop_setting32.delay;
		stop_setting->size = stop_setting32.size;

		reg_setting = compat_ptr(stop_setting32.reg_setting);

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}

	case CFG_SET_I2C_SYNC_PARAM: {
		struct msm_camera_cci_ctrl cci_ctrl;

		s_ctrl->sensor_i2c_client->cci_client->cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		s_ctrl->sensor_i2c_client->cci_client->id_map =
			cdata->cfg.sensor_i2c_sync_params.csid;

		CDBG("I2C_SYNC_PARAM CID:%d, line:%d delay:%d, cdid:%d\n",
			s_ctrl->sensor_i2c_client->cci_client->cid,
			cdata->cfg.sensor_i2c_sync_params.line,
			cdata->cfg.sensor_i2c_sync_params.delay,
			cdata->cfg.sensor_i2c_sync_params.csid);

		cci_ctrl.cmd = MSM_CCI_SET_SYNC_CID;
		cci_ctrl.cfg.cci_wait_sync_cfg.line =
			cdata->cfg.sensor_i2c_sync_params.line;
		cci_ctrl.cfg.cci_wait_sync_cfg.delay =
			cdata->cfg.sensor_i2c_sync_params.delay;
		cci_ctrl.cfg.cci_wait_sync_cfg.cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		cci_ctrl.cfg.cci_wait_sync_cfg.csid =
			cdata->cfg.sensor_i2c_sync_params.csid;
		rc = v4l2_subdev_call(s_ctrl->sensor_i2c_client->
				cci_client->cci_subdev,
				core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (rc < 0) {
			pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
			rc = -EFAULT;
			break;
		}
		break;
	}
	#if 1
	case CFG_I2C_IOCTL_R_OTP:
		if (s_ctrl->func_tbl->sensor_i2c_read_fuseid32 == NULL) {
			rc = -EFAULT;
			break;
		}
		rc = s_ctrl->func_tbl->sensor_i2c_read_fuseid32(cdata, s_ctrl);
	break;
#endif
#ifdef CONFIG_OIS_CALIBRATION
    case CFG_SET_GYRO_CALIBRATION:
        rc = htc_ois_calibration(s_ctrl, cdata->cam_id);
    break;
#endif
	default:
		rc = -EFAULT;
		break;
	}

DONE:
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
#endif

int msm_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
			cdata->cfg.sensor_info.subdev_intf[i] =
				s_ctrl->sensordata->sensor_info->subdev_intf[i];
		}
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
			CDBG("%s:%d subdev_intf[%d] %d\n", __func__, __LINE__,
				i, cdata->cfg.sensor_info.subdev_intf[i]);
		}
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;

	case CFG_WRITE_I2C_ARRAY:
	case CFG_WRITE_I2C_ARRAY_SYNC:
	case CFG_WRITE_I2C_ARRAY_SYNC_BLOCK:
	case CFG_WRITE_I2C_ARRAY_ASYNC: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		if (cdata->cfgtype == CFG_WRITE_I2C_ARRAY)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table(s_ctrl->sensor_i2c_client,
					&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_ASYNC == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_async(s_ctrl->sensor_i2c_client,
					&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_SYNC_BLOCK == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync_block(
					s_ctrl->sensor_i2c_client,
					&conf_array);
		else
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync(s_ctrl->sensor_i2c_client,
					&conf_array);

		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		struct msm_camera_i2c_read_config *read_config_ptr = NULL;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		read_config_ptr =
			(struct msm_camera_i2c_read_config *)cdata->cfg.setting;
		if (copy_from_user(&read_config, read_config_ptr,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		}
		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		read_config_ptr->data = local_data;
		break;
	}
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t write_slave_addr = 0;
		uint16_t orig_slave_addr = 0;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (copy_from_user(&write_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_array_write_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:CFG_SLAVE_WRITE_I2C_ARRAY:", __func__);
		CDBG("%s:slave_addr=0x%x, array_size=%d\n", __func__,
			write_config.slave_addr,
			write_config.conf_array.size);

		if (!write_config.conf_array.size ||
			write_config.conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(write_config.conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
				(void *)(write_config.conf_array.reg_setting),
				write_config.conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		write_config.conf_array.reg_setting = reg_setting;
		write_slave_addr = write_config.slave_addr;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				write_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				write_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				write_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &(write_config.conf_array));
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (copy_from_user(stop_setting,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}

	case CFG_SET_I2C_SYNC_PARAM: {
		struct msm_camera_cci_ctrl cci_ctrl;

		s_ctrl->sensor_i2c_client->cci_client->cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		s_ctrl->sensor_i2c_client->cci_client->id_map =
			cdata->cfg.sensor_i2c_sync_params.csid;

		CDBG("I2C_SYNC_PARAM CID:%d, line:%d delay:%d, cdid:%d\n",
			s_ctrl->sensor_i2c_client->cci_client->cid,
			cdata->cfg.sensor_i2c_sync_params.line,
			cdata->cfg.sensor_i2c_sync_params.delay,
			cdata->cfg.sensor_i2c_sync_params.csid);

		cci_ctrl.cmd = MSM_CCI_SET_SYNC_CID;
		cci_ctrl.cfg.cci_wait_sync_cfg.line =
			cdata->cfg.sensor_i2c_sync_params.line;
		cci_ctrl.cfg.cci_wait_sync_cfg.delay =
			cdata->cfg.sensor_i2c_sync_params.delay;
		cci_ctrl.cfg.cci_wait_sync_cfg.cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		cci_ctrl.cfg.cci_wait_sync_cfg.csid =
			cdata->cfg.sensor_i2c_sync_params.csid;
		rc = v4l2_subdev_call(s_ctrl->sensor_i2c_client->
				cci_client->cci_subdev,
				core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (rc < 0) {
			pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
			rc = -EFAULT;
			break;
		}
		break;
	}
	#if 1
	case CFG_I2C_IOCTL_R_OTP:
		if (s_ctrl->func_tbl->sensor_i2c_read_fuseid == NULL) {
			rc = -EFAULT;
			break;
		}
		rc = s_ctrl->func_tbl->sensor_i2c_read_fuseid(cdata, s_ctrl);
	break;
#endif
#ifdef CONFIG_OIS_CALIBRATION
    case CFG_SET_GYRO_CALIBRATION:
        rc = htc_ois_calibration(s_ctrl, cdata->cam_id);
    break;
#endif
	default:
		rc = -EFAULT;
		break;
	}

DONE:
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int msm_sensor_check_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0)
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static int msm_sensor_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (!on && s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}

static int msm_sensor_v4l2_enum_fmt(struct v4l2_subdev *sd,
	unsigned int index, enum v4l2_mbus_pixelcode *code)
{
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);

	if ((unsigned int)index >= s_ctrl->sensor_v4l2_subdev_info_size)
		return -EINVAL;

	*code = s_ctrl->sensor_v4l2_subdev_info[index].code;
	return 0;
}

static struct v4l2_subdev_core_ops msm_sensor_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops msm_sensor_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops msm_sensor_subdev_ops = {
	.core = &msm_sensor_subdev_core_ops,
	.video  = &msm_sensor_subdev_video_ops,
};

static struct msm_sensor_fn_t msm_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_i2c_read_fuseid = msm_sensor_read_fuseid,
#ifdef CONFIG_COMPAT
	.sensor_i2c_read_fuseid32 =msm_sensor_read_fuseid32,
#endif
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
	.i2c_write_table_async = msm_camera_cci_i2c_write_table_async,
	.i2c_write_table_sync = msm_camera_cci_i2c_write_table_sync,
	.i2c_write_table_sync_block = msm_camera_cci_i2c_write_table_sync_block,

};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq = msm_camera_qup_i2c_write_seq,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_write_conf_tbl = msm_camera_qup_i2c_write_conf_tbl,
	.i2c_write_table_async = msm_camera_qup_i2c_write_table,
	.i2c_write_table_sync = msm_camera_qup_i2c_write_table,
	.i2c_write_table_sync_block = msm_camera_qup_i2c_write_table,
};

int32_t msm_sensor_init_default_params(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t                       rc = -ENOMEM;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_cam_clk_info      *clk_info = NULL;
	unsigned long mount_pos = 0;

	
	if (!s_ctrl) {
		pr_err("%s:%d failed: invalid params s_ctrl %p\n", __func__,
			__LINE__, s_ctrl);
		return -EINVAL;
	}

	if (!s_ctrl->sensor_i2c_client) {
		pr_err("%s:%d failed: invalid params sensor_i2c_client %p\n",
			__func__, __LINE__, s_ctrl->sensor_i2c_client);
		return -EINVAL;
	}

	
	s_ctrl->sensor_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client->cci_client) {
		pr_err("%s:%d failed: no memory cci_client %p\n", __func__,
			__LINE__, s_ctrl->sensor_i2c_client->cci_client);
		return -ENOMEM;
	}

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = s_ctrl->sensor_i2c_client->cci_client;

		
		cci_client->cci_subdev = msm_cci_get_subdev();

		
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_cci_func_tbl;
	} else {
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl) {
			CDBG("%s:%d\n", __func__, __LINE__);
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_qup_func_tbl;
		}
	}

	
	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;

	
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;

	
	clk_info = kzalloc(sizeof(cam_8974_clk_info), GFP_KERNEL);
	if (!clk_info) {
		pr_err("%s:%d failed no memory clk_info %p\n", __func__,
			__LINE__, clk_info);
		rc = -ENOMEM;
		goto FREE_CCI_CLIENT;
	}
	memcpy(clk_info, cam_8974_clk_info, sizeof(cam_8974_clk_info));
	s_ctrl->sensordata->power_info.clk_info = clk_info;
	s_ctrl->sensordata->power_info.clk_info_size =
		ARRAY_SIZE(cam_8974_clk_info);

	
	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos | ((s_ctrl->sensordata->sensor_info->
					sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	return 0;

FREE_CCI_CLIENT:
	kfree(cci_client);
	return rc;
}

/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ANX_OHIO_DRV_H
#define _ANX_OHIO_DRV_H

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/async.h>
#include <linux/completion.h>

#include <linux/of_gpio.h>
#include <linux/of_platform.h>


#define LOG_TAG "Ohio"

#define ANX7418FW "usb_typec.img"
#define ANX7418FW_OTP_AB "usb_typec_otp.img"
#define ANX7418FW_OTP_AD "usb_typec_otp_ad.img"

#define DEVICE_ID_L 0x02
#define DEVICE_ID_H 0x03

#define DEVICE_VERSION 0x04
#define RESET_CTRL_0 0x05
#define POWER_DOWN_CTRL 0x0d
#define ANALOG_STATUS 0x40
#define ANALOG_CTRL_0 0x41
#define ANALOG_CTRL_1 0x42
#define ANALOG_CTRL_2 0x43
#define ANALOG_CTRL_3 0x44
#define ANALOG_CTRL_4 0x45
#define ANALOG_CTRL_5 0x46
#define ANALOG_CTRL_6 0x47
#define ANALOG_CTRL_7 0x48
#define ANALOG_CTRL_8 0x49
#define ANALOG_CTRL_9 0x4A
#define OHIO_SLVAVE_I2C_ADDR 0x50
#define EE_KEY_1 0x5D
#define EE_KEY_2 0x5E
#define EE_KEY_3 0x5F
#define R_EE_CTL_1 0x60
#define R_EE_CTL_2 0x61
#define R_EE_ADDR_L_BYTE 0x62
#define R_EE_ADDR_H_BYTE 0x63
#define R_EE_RD_DATA 0x65
#define R_EE_STATE 0x66
#define R_EE_BURST_DATA0 0x67
#define DEBUG_EE_0 0x7F
#define R_OTP_ADDR_HIGH 0xD0
#define R_OTP_ADDR_LOW 0xD1
#define R_OTP_DATA_IN_0 0xD2
#define R_OTP_ECC_IN 0xDB
#define R_OTP_DATA_OUT_0 0xDC
#define R_OTP_DATA_OUT_1 0xDD
#define R_OTP_DATA_OUT_2 0xDE
#define R_OTP_DATA_OUT_3 0xDF
#define R_OTP_DATA_OUT_4 0xE0
#define R_OTP_DATA_OUT_5 0xE1
#define R_OTP_DATA_OUT_6 0xE2
#define R_OTP_DATA_OUT_7 0xE3
#define R_OTP_DATA_OUT_8 0xE4
#define R_OTP_CTL_1 0xE5
#define R_OTP_STATE 0xED
#define R_OTP_ACC_PROTECT 0xEF

#define R_OCM_RESET (1 << 4)
#define R_ROM_LD_SPEED (3 << 6)
#define R_LOAD_EEPROM_NUM (2 << 2)
#define R_EE_WR_EN (1 << 0)
#define R_EE_BURST_WR_EN (1 << 4)
#define R_EE_RW_DONE (1 << 0)
#define DFP_OR_UFP (1 << 3)
#define SSRX_SWITCH_CC2 (1 << 4)
#define SSRX_SWITCH_CC1 (1 << 5)
#define SSTX_SWITCH_CC2 (1 << 6)
#define SSTX_SWITCH_CC1 (1 << 7)
#define R_MODE_TRANSITION (1 << 0)
#define R_EE_DEBUG_STATE (0xF << 0)
#define OTP_MODE (1 << 0)
#define R_POWER_DOWN_OCM (1 << 1)
#define R_MCU_ACCESS_DISABLE (1 << 7)
#define R_OTP_WAKEUP (1 << 5)
#define R_OTP_ECC_BYPASS (1 << 3)
#define R_OTP_WRITE (1 << 1)
#define R_OTP_READ (1 << 0)
#define R_OTP_READ_WORD_STATE (3 << 4)
#define R_OTP_WRITE_WORD_STATE (0xF << 0)
#define CC1_ATTACH (7 << 5)
#define CC2_ATTACH (7 << 2)
#define CC1_DETECT_RD (1 << 2)
#define CC1_DETECT_RA (3 << 2)
#define CC2_DETECT_RD (1 << 0)
#define CC2_DETECT_RA (3 << 0)

#define MAX_DEB_FAKE_IRQ_COUNT 10
#define OTP_PROGRAM_MAX 1
#define OTP_WRITE_RETRY 6
#define OTP_UPDATE_MAX 16
#define OTP_SIZE 0x3FFF

#define OTP_ACCESS_KEY 0x7A
#define EEPROM_ACCESS_KEY_1 0x28
#define EEPROM_ACCESS_KEY_2 0x5C
#define EEPROM_ACCESS_KEY_3 0x4E
#define FW_VERSION_OFFSET 4096
#define EEPROM_FW_HEADER_LEN 9
#define EEPROM_FW_VERSION_OFFSET EEPROM_FW_HEADER_LEN+FW_VERSION_OFFSET
#define OTP_FW_HEADER_LEN 27
#define OTP_FW_VERSION_ECC 512
#define OTP_FW_VERSION_OFFSET OTP_FW_HEADER_LEN+FW_VERSION_OFFSET+OTP_FW_VERSION_ECC
#define EEPROM_FW_CHIPID_LOW 1
#define EEPROM_FW_CHIPID_HIGH 0
#define OTP_FW_CHIPID_LOW 11
#define OTP_FW_CHIPID_HIGH 12
#define OTP_FW_HEADER_SIZE 20
#define OTP_FW_HEADER_START_ADDR 18
#define OTP_WRITE_DONE_RETRY 2000
#define OTP_READ_DONE_RETRY 2000

static unsigned char inactive_word[9] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
static unsigned char blank_word[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char ECC_table[] =
{
    0x0b, 0x3b, 0x37, 0x07, 0x19, 0x29, 0x49, 0x89, 
    0x16, 0x26, 0x46, 0x86, 0x13, 0x23, 0x43, 0x83, 
    0x1c, 0x2c, 0x4c, 0x8c, 0x15, 0x25, 0x45, 0x85, 
    0x1a, 0x2a, 0x4a, 0x8a, 0x0d, 0xcd, 0xce, 0x0e, 
    0x70, 0x73, 0xb3, 0xb0, 0x51, 0x52, 0x54, 0x58, 
    0xa1, 0xa2, 0xa4, 0xa8, 0x31, 0x32, 0x34, 0x38, 
    0xc1, 0xc2, 0xc4, 0xc8, 0x61, 0x62, 0x64, 0x68, 
    0x91, 0x92, 0x94, 0x98, 0xe0, 0xec, 0xdc, 0xd0  
};

enum ANX7418_chip_type {
	EEPROM,
	OTP_AB,
	OTP_AD,
	UNKNOWN_CHIP_TYPE,
};

enum ANX7418_fw_type {
	EEPROM_FW,
	OTP_AB_FW,
	OTP_AD_FW,
	UNKNOWN_FW_TYPE,
};

static const char *chip_type_to_string(enum ANX7418_chip_type type)
{
	switch (type) {
		case EEPROM:				return "EEPROM";
		case OTP_AB:				return "OTP AB";
		case OTP_AD:				return "OTP AD";
		case UNKNOWN_CHIP_TYPE:		return "UNKNOWN";
		default:					return "INVALID_TYPE";
	}
}

static const char *fw_type_to_string(enum ANX7418_fw_type type)
{
	switch (type) {
		case EEPROM_FW:				return "EEPROM";
		case OTP_AB_FW:				return "OTP AB";
		case OTP_AD_FW:				return "OTP AD";
		case UNKNOWN_FW_TYPE:		return "UNKNOWN";
		default:					return "INVALID_TYPE";
	}
}

bool ohio_is_connected(void);
void ohio_debug_dump(void);
unchar is_cable_detected(void);
extern u8 sel_voltage_pdo_index; 
extern u8 misc_status;
atomic_t ohio_power_status;
atomic_t cbl_det_irq_status;
void ohio_power_standby(void);
void ohio_hardware_poweron(void);
void ohio_hardware_powerdown(void);
void ohio_vbus_control(bool value);
#endif

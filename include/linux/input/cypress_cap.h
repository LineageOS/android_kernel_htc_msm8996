#ifndef CYPRESS_CAP_I2C_H
#define CYPRESS_CAP_I2C_H

#include <linux/types.h>

#define CYPRESS_CS_NAME			"CYPRESS_CAP"
#define CYPRESS_FW_UPDATE_NAME		"CYPRESS_FW_UPDATE"

#define CS_STATUS			(uint8_t) 0x00
#define CS_FW_VERSION			(uint8_t) 0x03
#define CS_PROJECT_ID			(uint8_t) 0x05
#define CS_SET_SLEEP			(uint8_t) 0x04
#define CS_ENTER_BL			(uint8_t) 0x06

#define CS_SENSITIVITY_START_REG	0x0A

#define CS_COMMON_REGISTER		12
#define CS_CP_START_REG			0x11
#define CS_RAW_START_REG		0x20
#define CS_BL_START_REG			0x2C
#define CS_FG_TH_START_REG		0x30
#define CS_NS_TH_START_REG		0x33

#define SUPPORT_CYPRESS_FW_UPDATE

#ifdef SUPPORT_CYPRESS_FW_UPDATE
#define SCNx8 "hhx"
#define MAX_FLASH_ARRAYS		4
#define NO_FLASH_ARRAY_DATA		0
#define TRANSFER_HEADER_SIZE		11
#define MAX_TRANDSFER_SIZE		64

#define CS_LENGTH_ID			5
#define CS_LENGTH_CHECKSUM		CS_LENGTH_ID + 1

#define MAX_COMMAND_SIZE		512
#define CMD_START			0x01
#define CMD_STOP			0x17
#define BASE_CMD_SIZE			0x07

#define CMD_VERIFY_CHECKSUM		0x31
#define CMD_GET_FLASH_SIZE		0x32
#define CMD_GET_APP_STATUS		0x33
#define CMD_ERASE_ROW			0x34
#define CMD_SYNC			0x35
#define CMD_SET_ACTIVE_APP		0x36
#define CMD_SEND_DATA			0x37
#define CMD_ENTER_BOOTLOADER		0x38
#define CMD_PROGRAM_ROW			0x39
#define CMD_VERIFY_ROW			0x3A
#define CMD_EXIT_BOOTLOADER		0x3B

#define CYRET_SUCCESS			0x00
#define CYRET_ERR_FILE			0x01
#define CYRET_ERR_EOF			0x02
#define CYRET_ERR_LENGTH		0x03
#define CYRET_ERR_DATA			0x04
#define CYRET_ERR_CMD			0x05
#define CYRET_ERR_DEVICE		0x06
#define CYRET_ERR_VERSION		0x07
#define CYRET_ERR_CHECKSUM		0x08
#define CYRET_ERR_ARRAY			0x09
#define CYRET_ERR_ROW			0x0A
#define CYRET_ERR_BTLDR			0x0B
#define CYRET_ERR_ACTIVE		0x0C
#define CYRET_ERR_UNK			0x0F
#define CYRET_ABORT			0xFF

typedef enum
{
	SUM_CHECKSUM = 0x00,
	CRC_CHECKSUM = 0x01,
} Cypress_ChecksumType;
#endif

struct info {
	uint16_t pid;
	uint16_t version;
};

struct cypress_bootloader_header {
	union {
		struct {
			unsigned long siliconID;
			unsigned char siliconRev;
			unsigned char packetChkSumType;
		} __packed;
		unsigned char data[CS_LENGTH_CHECKSUM];
	};
};

struct cypress_cap_platform_data {
	struct  info info;
	uint16_t gpio_rst;
	uint16_t gpio_irq;
	uint16_t gpio_ind;
	uint16_t gpio_id;
	uint16_t gpio_id_active_low;
	int     (*power)(int on);
	int     (*reset)(struct i2c_client *client);
	uint8_t cap_num;
	uint8_t raw_num;
	int     keycode[4];
	int     update_feature;
	void    (*gpio_init)(void);
	uint32_t cap_regulator_volt;
	struct regulator *cap_regulator;
	struct pinctrl *cap_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

#endif //CYPRESS_CAP_I2C_H

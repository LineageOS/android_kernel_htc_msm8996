/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */


#include "inc/vl53l0_i2c.h"


#define STATUS_OK              0x00
#define STATUS_FAIL            0x01


extern int Laser_RegWriteByte(u8 addr, u8 data);
extern int Laser_RegReadByte(u8 addr, u8 *data);
extern int Laser_RegWriteWord(u8 addr, u16 data);
extern int Laser_RegReadWord(u8 addr, u16 *data);
extern int Laser_RegWriteDWord(u8 addr, u32 data);
extern int Laser_RegReadDWord(u8 addr, u32 *data);
extern int Laser_RegReadMulti(u8 addr, u8 *data, uint32_t count);


int32_t VL53L0_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;


    status = Laser_RegReadMulti(index, pdata, count);

    return status;
}


int32_t VL53L0_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;

    status = Laser_RegWriteByte(index, data);

    return status;

}


int32_t VL53L0_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;


    

    status = Laser_RegWriteWord(index, data);

    return status;

}


int32_t VL53L0_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;

    

    status = Laser_RegWriteDWord(index, data);

    return status;

}


int32_t VL53L0_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int status = STATUS_OK;

    status = Laser_RegReadByte(index, pdata);

    return status;

}


int32_t VL53L0_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;

    status = Laser_RegReadWord(index, pdata);

    return status;

}

int32_t VL53L0_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;

    status = Laser_RegReadDWord(index, pdata);

    return status;

}



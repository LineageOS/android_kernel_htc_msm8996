/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

int RamWrite32A( unsigned short RamAddr, unsigned int RamData );
int RamRead32A( unsigned short RamAddr, unsigned int * ReadData );
unsigned short MakeNVRSel( unsigned short UsAddress );
unsigned int MakeNVRDat( unsigned short UsAddress, unsigned char UcData );
unsigned char MakeDatNVR( unsigned short UsAddress, unsigned int UlData );
void WPBCtrl( unsigned char UcCtrl );
unsigned char htc_GyroReCalib(struct msm_sensor_ctrl_t *s_ctrl, int cam_id);
short htc_WrGyroOffsetData( void );
int GYRO_Cali_release(void);
signed short Calibration_VerifyUpdate_PreRead( void );
void FlashResetRelease(void);
void FlashAutoConfig( void );
signed short Calibration_VerifyUpdate( void );
void IOWrite32A( unsigned int IOadrs, unsigned int IOdata );
void IORead4times32A( unsigned int* Dat );
unsigned char ReadWPB( void );
void IORead32A( unsigned int IOadrs, unsigned int *IOdata );
void CRC16_main( unsigned char *p, int Num );
void IOWriteDouble32A( unsigned int IOadrs1, unsigned int IOdata1, unsigned int IOadrs2, unsigned int IOdata2 );
int CntWrt( unsigned char * PcSetDat, unsigned short CntWrt);
int CntRd( unsigned int addr, unsigned char *	PcSetDat, unsigned short	UsDatNum );
int htc_checkFWUpdate(struct msm_sensor_ctrl_t *s_ctrl);
signed short FlashWrite_CalibID( const unsigned int CalibId );


struct GYRO_gpio_info{
    int flash_rw;
};

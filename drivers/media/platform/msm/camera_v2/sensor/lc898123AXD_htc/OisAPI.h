/**
 * @brief		OIS system header for LC898123
 * 				API List for customers
 *
 * @author		Copyright (C) 2015, ON Semiconductor, all right reserved.
 *
 * @file		OisAPI.h
 * @date		svn:$Date:: 2016-01-18 10:08:46 +0900#$
 * @revision	svn:$Revision: 115 $
 * @attention
 **/
#include	"MeasurementLibrary.h"

#ifdef	__OISCMD_5BSC02P1__
	#define	__OIS_CMD_HEADER__
#else
	#define	__OIS_CMD_HEADER__		extern
#endif

#define		INT16	signed short
#define		INT32	long
#define		INT64	long long
#define		UINT8	unsigned char
#define		UINT16	unsigned short
#define		UINT32	unsigned int
#define		UINT64	unsigned long long

#define			__OIS_MODULE_CALIBRATION__		


typedef struct STRECALIB {
	INT16	SsFctryOffX ;
	INT16	SsFctryOffY ;
	INT16	SsRecalOffX ;
	INT16	SsRecalOffY ;
	INT16	SsDiffX ;
	INT16	SsDiffY ;
} stReCalib ;

__OIS_CMD_HEADER__ void		WitTim( unsigned short );
__OIS_CMD_HEADER__ void		FlashReset(void);	
__OIS_CMD_HEADER__	unsigned short MakeNVRSelIdx( unsigned short UsAddress );


__OIS_CMD_HEADER__ void	SetTregAf( UINT16 UsTregAf );				
 

__OIS_CMD_HEADER__	UINT8	RdStatus( UINT8 UcStBitChk );			
__OIS_CMD_HEADER__	void	OisEna( void );							

__OIS_CMD_HEADER__	UINT8	RtnCen( UINT8	UcCmdPar ) ;			
__OIS_CMD_HEADER__	void	SetPanTiltMode( UINT8 UcPnTmod );		
__OIS_CMD_HEADER__	void	RdHallCalData( void );					

__OIS_CMD_HEADER__  void	SetSinWavePara( UINT8 , UINT8 ) ;		

__OIS_CMD_HEADER__	UINT8	RunHea( void ) ;						
__OIS_CMD_HEADER__	UINT8	RunGea( void ) ;						
__OIS_CMD_HEADER__	UINT8	RunGea2( UINT8 ) ;						

__OIS_CMD_HEADER__	INT16	FlashCheck_MainVerify( void );			
__OIS_CMD_HEADER__	INT16	FlashCheck_NVRVerify( void ); 			
__OIS_CMD_HEADER__	INT16	FlashNVR_WriteData_Para( UINT16, UINT8 *, UINT16 );


__OIS_CMD_HEADER__	UINT32	GetFromVer( void );
__OIS_CMD_HEADER__	void	OscStb( void );							
__OIS_CMD_HEADER__	UINT8	FrqDet( void );							
__OIS_CMD_HEADER__	void	MeasureGyroAmp( void );					
__OIS_CMD_HEADER__	void	GetGyroWhoAmI( UINT8 * ) ;				
__OIS_CMD_HEADER__	void	GetGyroID( UINT8 * ) ;					
__OIS_CMD_HEADER__	void	GyroSleep( UINT8 ) ;					
__OIS_CMD_HEADER__	UINT8	GyroReCalib( stReCalib * ) ;			
__OIS_CMD_HEADER__	UINT32	ReadCalibID( void ) ;					

#ifdef	__OIS_MODULE_CALIBRATION__

 
 #ifdef	__OIS_CLOSED_AF__
 __OIS_CMD_HEADER__	UINT32	TneRunA( void );						
 #else
 __OIS_CMD_HEADER__	UINT32	TneRun( void );							
 #endif
 __OIS_CMD_HEADER__	void	IniNvc( INT16, INT16 ) ;				
 __OIS_CMD_HEADER__	void	TneSltPos( UINT8 ) ;					
 __OIS_CMD_HEADER__	void	TneVrtPos( UINT8 ) ;					
 __OIS_CMD_HEADER__ void	TneHrzPos( UINT8 ) ;					
 __OIS_CMD_HEADER__ UINT16	TneADO( void ) ;

 __OIS_CMD_HEADER__	INT16	WrHallCalData( void );					
 __OIS_CMD_HEADER__	INT16	WrGyroGainData( void );					
 __OIS_CMD_HEADER__	INT16	WrGyroGainData_NV( UINT32 , UINT32 ) ;	
 __OIS_CMD_HEADER__	INT16	WrMixCalData( UINT8, mlMixingValue * ) ;
 __OIS_CMD_HEADER__	INT16	WrMixGyroData( UINT16, UINT16, UINT16, UINT16 );
 __OIS_CMD_HEADER__	INT16	WrGyroOffsetData( void );
 __OIS_CMD_HEADER__	INT16	WrLinCalData( UINT8, mlLinearityValue * );
 __OIS_CMD_HEADER__	INT16	ErCalData( UINT16 );

 
 __OIS_CMD_HEADER__	INT16	FlashUpdate( void ) ;					
 __OIS_CMD_HEADER__	INT16	FlashUpdateM( void ) ;					
 __OIS_CMD_HEADER__	INT16	FlashUpdateF( void ) ;					
 __OIS_CMD_HEADER__	INT16	FlashWrite_NVRVerify( void );			
 __OIS_CMD_HEADER__	UINT8	ReadWPB( void );						
 __OIS_CMD_HEADER__	INT16 	FlashNVRSectorErase_ByteA( UINT16 );
 __OIS_CMD_HEADER__	INT16 	FlashNVRSectorErase_Byte( UINT8 );
 __OIS_CMD_HEADER__	void 	FlashNVR_ReadData_ByteA( UINT16, UINT8 *, UINT16 );
 __OIS_CMD_HEADER__	void 	FlashNVR_ReadData_Byte( UINT8, UINT8, UINT8 *, UINT16 );
 __OIS_CMD_HEADER__	INT16 	FlashNVR_WriteData_ByteA( UINT16, UINT8 *, UINT16 );
 __OIS_CMD_HEADER__	INT16 	FlashNVR_WriteData_Byte( UINT8, UINT8, UINT8 *, UINT16 );
 __OIS_CMD_HEADER__	void	FlashReadData_3B( UINT32 , UINT32 * , UINT16 );
 __OIS_CMD_HEADER__	INT16	FlashWriteData_3B( UINT32, UINT32 * , UINT16 );
 __OIS_CMD_HEADER__	void 	FlashMainArray_ReadData_Byte( UINT16, UINT8 *, UINT16);
 __OIS_CMD_HEADER__	INT16 	FlashMainArray_WriteData_Byte( UINT16, UINT8 *, UINT16);
 __OIS_CMD_HEADER__	INT16 	FlashMainArraySectorErase_Byte( UINT16 );
 __OIS_CMD_HEADER__	INT16	FlashMainArrayVerify_Byte( UINT16, UINT8 *, UINT16 );
 __OIS_CMD_HEADER__	INT16	FlashNVRVerify_Byte( UINT16, UINT8 *, UINT16 );
#endif	


__OIS_CMD_HEADER__ void		OisEnaNCL( void ) ;						
__OIS_CMD_HEADER__ void		OisEnaDrCl( void ) ;					
__OIS_CMD_HEADER__ void		OisEnaDrNcl( void ) ;					
__OIS_CMD_HEADER__ void		OisDis( void ) ;						

__OIS_CMD_HEADER__ void		SetRec( void ) ;						
__OIS_CMD_HEADER__ void		SetStill( void ) ;						
__OIS_CMD_HEADER__ void		SetRecPreview( UINT8 ) ;				
__OIS_CMD_HEADER__ void		SetStillPreview( UINT8 ) ;				
__OIS_CMD_HEADER__ void		TimPro( void ) ;						

__OIS_CMD_HEADER__ UINT16	RdFwVr( void ) ;						

#ifdef	HF_LINEAR_ENA
 __OIS_CMD_HEADER__	void	SetHalLnData( UINT16 * );
 __OIS_CMD_HEADER__	INT16	WrHalLnData( UINT8 );
#endif	




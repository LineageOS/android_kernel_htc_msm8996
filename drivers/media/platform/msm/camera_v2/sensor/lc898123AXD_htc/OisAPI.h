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

//****************************************************
//	extern selector for API
//****************************************************
#ifdef	__OISCMD_5BSC02P1__
	#define	__OIS_CMD_HEADER__
#else
	#define	__OIS_CMD_HEADER__		extern
#endif

/*HTC_START*/
#define		INT16	signed short
#define		INT32	long
#define		INT64	long long
#define		UINT8	unsigned char
#define		UINT16	unsigned short
#define		UINT32	unsigned int
#define		UINT64	unsigned long long
/*HTC_END*/

//****************************************************
//	MODE SELECTORS (Compile Switches)
//****************************************************
#define			__OIS_MODULE_CALIBRATION__		//!< for module maker to done the calibration. 
//#define 	   	__OIS_TYPE_XC__					//!< for LC898123XC
//#define 		__CRC_VERIFY__					//!< select CRC16 for upload verify, if this comment out, MD5 is selected.
//#define		__OIS_BIG_ENDIAN__				//!< endian of MPU

//#define		__OIS_CLOSED_AF__

//****************************************************
//	STRUCTURE DEFINE	
//****************************************************
typedef struct STRECALIB {
	INT16	SsFctryOffX ;
	INT16	SsFctryOffY ;
	INT16	SsRecalOffX ;
	INT16	SsRecalOffY ;
	INT16	SsDiffX ;
	INT16	SsDiffY ;
} stReCalib ;

//****************************************************
//	API FUNCTION LIST	
//****************************************************
/*HTC_START*/
__OIS_CMD_HEADER__ void		WitTim( unsigned short );
__OIS_CMD_HEADER__ void		FlashReset(void);	
__OIS_CMD_HEADER__	unsigned short MakeNVRSelIdx( unsigned short UsAddress );


/*HTC_END*/
/* AF position Control [mandatory] */
__OIS_CMD_HEADER__ void	SetTregAf( UINT16 UsTregAf );				/*!< Bi-direction :  Min:000h Max:7FFh (11bit) 
																	     Uni-direction : Min:000h Max:3FFh (10bit) */ 

/* Status Read and OIS enable [mandatory] */
__OIS_CMD_HEADER__	UINT8	RdStatus( UINT8 UcStBitChk );			//!< Status Read whether initialization finish or not.
__OIS_CMD_HEADER__	void	OisEna( void );							//!< OIS Enable Function

/* Others [option] */
__OIS_CMD_HEADER__	UINT8	RtnCen( UINT8	UcCmdPar ) ;			//!< Return to Center Function. Hall servo on/off
__OIS_CMD_HEADER__	void	SetPanTiltMode( UINT8 UcPnTmod );		//!< default ON.
__OIS_CMD_HEADER__	void	RdHallCalData( void );					//!< Read Hall Calibration Data in Data Ram

__OIS_CMD_HEADER__  void	SetSinWavePara( UINT8 , UINT8 ) ;		//!< Sin wave Test Function

__OIS_CMD_HEADER__	UINT8	RunHea( void ) ;						//!< Hall Examination of Acceptance
__OIS_CMD_HEADER__	UINT8	RunGea( void ) ;						//!< Gyro Examination of Acceptance
__OIS_CMD_HEADER__	UINT8	RunGea2( UINT8 ) ;						//!< Gyro Examination of Acceptance

__OIS_CMD_HEADER__	INT16	FlashCheck_MainVerify( void );			//!< Flash Code verification
__OIS_CMD_HEADER__	INT16	FlashCheck_NVRVerify( void ); 			//!< NVR data verification
__OIS_CMD_HEADER__	INT16	FlashNVR_WriteData_Para( UINT16, UINT8 *, UINT16 );


__OIS_CMD_HEADER__	UINT32	GetFromVer( void );
__OIS_CMD_HEADER__	void	OscStb( void );							//!< Standby the oscillator
__OIS_CMD_HEADER__	UINT8	FrqDet( void );							//!< Detection of small oscillation.
__OIS_CMD_HEADER__	void	MeasureGyroAmp( void );					//!< Measurement the amplitude of gyro @6Hz/1degree
__OIS_CMD_HEADER__	void	GetGyroWhoAmI( UINT8 * ) ;				//!< Get Gyro Who Am I
__OIS_CMD_HEADER__	void	GetGyroID( UINT8 * ) ;					//!< Get Gyro ID
__OIS_CMD_HEADER__	void	GyroSleep( UINT8 ) ;					//!< Gyro Sleep Control
__OIS_CMD_HEADER__	UINT8	GyroReCalib( stReCalib * ) ;			//!< Gyro offset re-calibration
__OIS_CMD_HEADER__	UINT32	ReadCalibID( void ) ;					//!< Read calibration ID

#ifdef	__OIS_MODULE_CALIBRATION__

 /* Calibration Main [mandatory] */
 #ifdef	__OIS_CLOSED_AF__
 __OIS_CMD_HEADER__	UINT32	TneRunA( void );						//!< calibration with close AF
 #else
 __OIS_CMD_HEADER__	UINT32	TneRun( void );							//!< calibration for bi-direction AF
 #endif
 __OIS_CMD_HEADER__	void	IniNvc( INT16, INT16 ) ;				//!< for NVC
 __OIS_CMD_HEADER__	void	TneSltPos( UINT8 ) ;					//!< for NVC
 __OIS_CMD_HEADER__	void	TneVrtPos( UINT8 ) ;					//!< for NVC
 __OIS_CMD_HEADER__ void	TneHrzPos( UINT8 ) ;					//!< for NVC
 __OIS_CMD_HEADER__ UINT16	TneADO( void ) ;

 __OIS_CMD_HEADER__	INT16	WrHallCalData( void );					//!< upload the calibration data except gyro gain to Flash
 __OIS_CMD_HEADER__	INT16	WrGyroGainData( void );					//!< upload the gyro gain to Flash
 __OIS_CMD_HEADER__	INT16	WrGyroGainData_NV( UINT32 , UINT32 ) ;	//!< upload the gyro gain with parameter to Flash
 __OIS_CMD_HEADER__	INT16	WrMixCalData( UINT8, mlMixingValue * ) ;//!< upload the mixing coefficient to Flash
 __OIS_CMD_HEADER__	INT16	WrMixGyroData( UINT16, UINT16, UINT16, UINT16 );
 __OIS_CMD_HEADER__	INT16	WrGyroOffsetData( void );
 __OIS_CMD_HEADER__	INT16	WrLinCalData( UINT8, mlLinearityValue * );
 __OIS_CMD_HEADER__	INT16	ErCalData( UINT16 );

 /* Flash Update */
 __OIS_CMD_HEADER__	INT16	FlashUpdate( void ) ;					//!< upload DSP code for main to Flash
 __OIS_CMD_HEADER__	INT16	FlashUpdateM( void ) ;					//!< upload DSP code for main to Flash
 __OIS_CMD_HEADER__	INT16	FlashUpdateF( void ) ;					//!< upload DSP code for front to Flash
 __OIS_CMD_HEADER__	INT16	FlashWrite_NVRVerify( void );			//!< updata the NVR data verifyed code after calibration 
 __OIS_CMD_HEADER__	UINT8	ReadWPB( void );						//!< read state of WPB pin
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
#endif	// __OIS_MODULE_CALIBRATION__


__OIS_CMD_HEADER__ void		OisEnaNCL( void ) ;						//!< OIS Enable Function w/o delay clear
__OIS_CMD_HEADER__ void		OisEnaDrCl( void ) ;					//!< OIS Enable Function with drift control with delay clear
__OIS_CMD_HEADER__ void		OisEnaDrNcl( void ) ;					//!< OIS Enable Function with drift control w/o delay clear
__OIS_CMD_HEADER__ void		OisDis( void ) ;						//!< OIS Disable Function

__OIS_CMD_HEADER__ void		SetRec( void ) ;						//!< Rec Mode Enable Function
__OIS_CMD_HEADER__ void		SetStill( void ) ;						//!< Still Mode Enable Function
__OIS_CMD_HEADER__ void		SetRecPreview( UINT8 ) ;				//!< Rec Preview Mode Enable Function
__OIS_CMD_HEADER__ void		SetStillPreview( UINT8 ) ;				//!< Still Preview Mode Enable Function
__OIS_CMD_HEADER__ void		TimPro( void ) ;						//!< Timer Interrupt Process Function

__OIS_CMD_HEADER__ UINT16	RdFwVr( void ) ;						//!< Read Fw Version Function

#ifdef	HF_LINEAR_ENA
 __OIS_CMD_HEADER__	void	SetHalLnData( UINT16 * );
 __OIS_CMD_HEADER__	INT16	WrHalLnData( UINT8 );
#endif	// HF_LINEAR_ENA




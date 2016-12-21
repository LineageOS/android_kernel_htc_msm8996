/**
 * @brief		OIS system common header for LC898123
 * 				Defines, Structures, function prototypes
 *
 * @author		Copyright (C) 2015, ON Semiconductor, all right reserved.
 *
 * @file		Ois.h
 * @date		svn:$Date:: 2016-03-29 18:23:44 +0900#$
 * @revision	svn:$Revision: 120 $
 * @attention
 **/
#if 0
#define		INT16	short
#define		INT32	long
#define		INT64	long long
#define		UINT8	unsigned char
#define		UINT16	unsigned short
#define		UINT32	unsigned long
#define		UINT64	unsigned long long
#else
#define		INT16	signed short
#define		INT32	long
#define		INT64	long long
#define		UINT8	unsigned char
#define		UINT16	unsigned short
#define		UINT32	unsigned int
#define		UINT64	unsigned long long
#endif

 
#define		NEUTRAL_CENTER					
#define		NEUTRAL_CENTER_FINE				

#define		HF_LINEAR_ENA					
#define		HF_MIXING_ENA

#include "OisAPI.h"
#ifdef	__OIS_TYPE_XC__						
#include "OisLc898123XC.h"
#else
#include "OisLc898123AXD.h"
#endif

 #define	MDL_VER			0x01
 #define	FW_VER			0x0F

#define		EXE_END		0x00000002L			
#define		EXE_HXADJ	0x00000006L			
#define		EXE_HYADJ	0x0000000AL			
#define		EXE_LXADJ	0x00000012L			
#define		EXE_LYADJ	0x00000022L			
#define		EXE_GXADJ	0x00000042L			
#define		EXE_GYADJ	0x00000082L			
#define		EXE_ERR		0x00000099L			
#ifdef	SEL_CLOSED_AF
 #define	EXE_HZADJ	0x00100002L			
 #define	EXE_LZADJ	0x00200002L			
#endif

#define		EXE_HXMVER	0x06				
#define		EXE_HYMVER	0x0A				

#define		EXE_GXABOVE	0x06				
#define		EXE_GXBELOW	0x0A				
#define		EXE_GYABOVE	0x12				
#define		EXE_GYBELOW	0x22				

#define	SUCCESS			0x00				
#define	FAILURE			0x01				

#ifndef ON
 #define	ON				0x01			
 #define	OFF				0x00			
#endif
#define	SPC				0x02				

#define	X_DIR			0x00				
#define	Y_DIR			0x01				
#define	Z_DIR			0x02				

#define	NOP_TIME		0.00004166F

#define	WPB_OFF			0x01				
#define WPB_ON			0x00				

#define	MD5_MAIN		0x01				
#define MD5_NVR			0x02				
#define MD5_BOTH		(MD5_MAIN | MD5_NVR)

#define		SXGAIN_LOP		0x30000000
#define		SYGAIN_LOP		0x30000000
#define		XY_BIAS			0x40000000
#define		XY_OFST			0x80000000
#ifdef	SEL_CLOSED_AF
#define		SZGAIN_LOP		0x30000000
#define		Z_BIAS			0x40000000
#define		Z_OFST			0x80000000
#endif

struct STFILREG {
	UINT16	UsRegAdd ;
	UINT8	UcRegDat ;
} ;											

struct STFILRAM {
	UINT16	UsRamAdd ;
	UINT32	UlRamDat ;
} ;											

struct STCMDTBL {
	UINT16	Cmd ;
	UINT32	UiCmdStf ;
	void ( *UcCmdPtr )( void ) ;
} ;

#define		CMD_IO_ADR_ACCESS				0xC000				
#define		CMD_IO_DAT_ACCESS				0xD000				
#define		CMD_REMAP						0xF001				
#define		CMD_REBOOT						0xF003				
#define		CMD_RETURN_TO_CENTER			0xF010				
	#define		BOTH_SRV_OFF					0x00000000			
	#define		XAXS_SRV_ON						0x00000001			
	#define		YAXS_SRV_ON						0x00000002			
	#define		BOTH_SRV_ON						0x00000003			
	#define		ZAXS_SRV_OFF					0x00000004			
	#define		ZAXS_SRV_ON						0x00000005			
#define		CMD_PAN_TILT					0xF011				
	#define		PAN_TILT_OFF					0x00000000			
	#define		PAN_TILT_ON						0x00000001			
#define		CMD_OIS_ENABLE					0xF012				
	#define		OIS_DISABLE						0x00000000			
	#define		OIS_ENABLE						0x00000001			
	#define		OIS_ENA_NCL						0x00000002			
	#define		OIS_ENA_DOF						0x00000004			
#define		CMD_MOVE_STILL_MODE				0xF013				
	#define		MOVIE_MODE						0x00000000			
	#define		STILL_MODE						0x00000001			
	#define		MOVIE_MODE1						0x00000002			
	#define		STILL_MODE1						0x00000003			
	#define		MOVIE_MODE2						0x00000004			
	#define		STILL_MODE2						0x00000005			
	#define		MOVIE_MODE3						0x00000006			
	#define		STILL_MODE3						0x00000007			
#define		CMD_CALIBRATION					0xF014				
#define		CMD_CHASE_CONFIRMATION			0xF015				
#define		CMD_GYRO_SIG_CONFIRMATION		0xF016				
#define		CMD_FLASH_LOAD					0xF017				
	#define		HALL_CALB_FLG					0x00008000			
		#define		HALL_CALB_BIT					0x00FF00FF
	#define		GYRO_GAIN_FLG					0x00004000			
	#define		ANGL_CORR_FLG					0x00002000			
	#define		FOCL_GAIN_FLG					0x00001000			
	#define		CLAF_CALB_FLG					0x00000800			
	#define		HLLN_CALB_FLG					0x00000400			
	#define		CROS_TALK_FLG					0x00000200			
#define		CMD_GYRO_RD_ACCS				0xF01D				
#define		CMD_GYRO_WR_ACCS				0xF01E				

#define		CMD_READ_STATUS					0xF100				

#define		READ_STATUS_INI					0x01000000

#define		STBOSCPLL						0x00D00074			
	#define		OSC_STB							0x00000002			

#define CmEqSw1			0					
#define CmEqSw2			1					
#define CmShakeOn		2					
#define CmRecMod		4					
#define CmCofCnt		5					
#define CmTpdCnt		6					
#define CmIntDrft		7					
#define CmAfZoom		0					



#define	HLXO			0x00000001			
#define	HLYO			0x00000002			
#define	HLXBO			0x00000004			
#define	HLYBO			0x00000008			
#define	HLAFO			0x00000010			
#define	HLAFBO			0x00000020			



typedef struct {
	INT32				SiSampleNum ;		
	INT32				SiSampleMax ;		

	struct {
		INT32			SiMax1 ;			
		INT32			SiMin1 ;			
		UINT32	UiAmp1 ;					
		INT64		LLiIntegral1 ;			
		INT64		LLiAbsInteg1 ;			
		INT32			PiMeasureRam1 ;		
	} MeasureFilterA ;

	struct {
		INT32			SiMax2 ;				
		INT32			SiMin2 ;				
		UINT32	UiAmp2 ;					
		INT64		LLiIntegral2 ;			
		INT64		LLiAbsInteg2 ;			
		INT32			PiMeasureRam2 ;		
	} MeasureFilterB ;
} MeasureFunction_Type ;


#ifdef __OIS_BIG_ENDIAN__
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcHigVal ;
		UINT8	UcLowVal ;
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa3 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa0 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlHigVal ;
		UINT32	UlLowVal ;
	} StUllnVal ;
} ;


union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsHigVal ;
		UINT16	UsLowVal ;
	} StFltVal ;
} ;

#else	
union	WRDVAL{
	INT16	SsWrdVal ;
	UINT16	UsWrdVal ;
	UINT8	UcWrkVal[ 2 ] ;
	signed char		ScWrkVal[ 2 ] ;
	struct {
		UINT8	UcLowVal ;
		UINT8	UcHigVal ;
	} StWrdVal ;
} ;


union	DWDVAL {
	UINT32	UlDwdVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT8	UcRamVa0 ;
		UINT8	UcRamVa1 ;
		UINT8	UcRamVa2 ;
		UINT8	UcRamVa3 ;
	} StCdwVal ;
} ;

union	ULLNVAL {
	UINT64	UllnValue ;
	UINT32	UlnValue[ 2 ] ;
	struct {
		UINT32	UlLowVal ;
		UINT32	UlHigVal ;
	} StUllnVal ;
} ;

union	FLTVAL {
	float			SfFltVal ;
	UINT32	UlLngVal ;
	UINT16	UsDwdVal[ 2 ] ;
	struct {
		UINT16	UsLowVal ;
		UINT16	UsHigVal ;
	} StFltVal ;
} ;
#endif	

typedef union WRDVAL	UnWrdVal ;
typedef union DWDVAL	UnDwdVal;
typedef union ULLNVAL	UnllnVal;
typedef union FLTVAL	UnFltVal ;


typedef struct STADJPAR {
	struct {
		UINT32	UlAdjPhs ;					

		UINT16	UsHlxCna ;					
		UINT16	UsHlxMax ;					
		UINT16	UsHlxMxa ;					
		UINT16	UsHlxMin ;					
		UINT16	UsHlxMna ;					
		UINT16	UsHlxGan ;					
		UINT16	UsHlxOff ;					
		UINT16	UsAdxOff ;					
		UINT16	UsHlxCen ;					

		UINT16	UsHlyCna ;					
		UINT16	UsHlyMax ;					
		UINT16	UsHlyMxa ;					
		UINT16	UsHlyMin ;					
		UINT16	UsHlyMna ;					
		UINT16	UsHlyGan ;					
		UINT16	UsHlyOff ;					
		UINT16	UsAdyOff ;					
		UINT16	UsHlyCen ;					

#ifdef	SEL_CLOSED_AF
		UINT16	UsHlzCna ;					
		UINT16	UsHlzMax ;					
		UINT16	UsHlzMxa ;					
		UINT16	UsHlzMin ;					
		UINT16	UsHlzMna ;					
		UINT16	UsHlzGan ;					
		UINT16	UsHlzOff ;					
		UINT16	UsAdzOff ;					
		UINT16	UsHlzCen ;					
#endif
	} StHalAdj ;

	struct {
		UINT32	UlLxgVal ;					
		UINT32	UlLygVal ;					
#ifdef	SEL_CLOSED_AF
		UINT32	UlLzgVal ;					
#endif
	} StLopGan ;

	struct {
		UINT16	UsGxoVal ;					
		UINT16	UsGyoVal ;					
		UINT16	UsGxoSts ;					
		UINT16	UsGyoSts ;					
	} StGvcOff ;
	
	UINT8		UcOscVal ;					

} stAdjPar ;

__OIS_CMD_HEADER__	stAdjPar	StAdjPar ;	


typedef struct STMESRAM {
	INT32	SlMeasureMaxValue ;
	INT32	SlMeasureMinValue ;
	INT32	SlMeasureAmpValue ;
	INT32	SlMeasureAveValue ;
} stMesRam ;								

typedef struct STHALLINEAR {
	UINT16	XCoefA[6] ;
	UINT16	XCoefB[6] ;
	UINT16	XZone[5] ;
	UINT16	YCoefA[6] ;
	UINT16	YCoefB[6] ;
	UINT16	YZone[5] ;
} stHalLinear ;								

#define		BOTH_ON			0x00
#define		XONLY_ON		0x01
#define		YONLY_ON		0x02
#define		BOTH_OFF		0x03
#define		ZONLY_OFF		0x04
#define		ZONLY_ON		0x05
#define		SINEWAVE		0
#define		XHALWAVE		1
#define		YHALWAVE		2
#define		ZHALWAVE		3
#define		XACTTEST		10
#define		YACTTEST		11
#define		CIRCWAVE		255
#define		HALL_H_VAL		0x3F800000		
#define		PTP_BEFORE		0
#define		PTP_AFTER		1
#define		PTP_ACCEPT		2
#define		ACT_CHK_LVL		0x33320000		
#define		ACT_CHK_FRQ		0x00068C16		
#define		ACT_CHK_NUM		5005			
#define		ACT_THR			0x0A3D0000		
#define		GEA_DIF_HIG		0x0057			
#define		GEA_DIF_LOW		0x0001
#define		GEA_MAX_LVL		0x0A41			
#define		GEA_MIN_LVL		0x1482			
#define		GEA_MINMAX_MODE	0x00			
#define		GEA_MEAN_MODE	0x01			

#define _GET_UINT32(n,b)				\
{										\
	(n) = ( (UINT32) (b)[0]       )		\
		| ( (UINT32) (b)[1] <<  8 )		\
		| ( (UINT32) (b)[2] << 16 )		\
		| ( (UINT32) (b)[3] << 24 );	\
}

#define _PUT_UINT32(n,b)				\
{										\
	(b)[0] = (UINT8) ( (n)       );		\
	(b)[1] = (UINT8) ( (n) >>  8 );		\
	(b)[2] = (UINT8) ( (n) >> 16 );		\
	(b)[3] = (UINT8) ( (n) >> 24 );		\
}
#define _GET_UINT16(n,b)				\
{										\
	(n) = ( (UINT16) (b)[0]       )		\
		| ( (UINT16) (b)[1] <<  8 );	\
}

#define _PUT_UINT16(n,b)				\
{										\
	(b)[0] = (UINT8) ( (n)       );		\
	(b)[1] = (UINT8) ( (n) >>  8 );		\
}

#define _GET_UINT16BIG(n,b)				\
{										\
	(n) = ( (UINT16) (b)[1]       )		\
		| ( (UINT16) (b)[0] <<  8 );	\
}

#define _PUT_UINT16BIG(n,b)				\
{										\
	(b)[1] = (UINT8) ( (n)       );		\
	(b)[0] = (UINT8) ( (n) >>  8 );		\
}

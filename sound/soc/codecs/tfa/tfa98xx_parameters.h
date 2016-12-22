
#ifndef TFA98XXPARAMETERS_H_
#define TFA98XXPARAMETERS_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "tfa_service.h"

#if (defined(WIN32) || defined(_X64))
#pragma warning(disable:4200)
#pragma warning(disable:4214)
#endif

#define TFA_MAX_PROFILES                (64)
#define TFA_MAX_VSTEPS                  (64)
#define TFA_MAX_VSTEP_MSG_MARKER        (100) /* This marker  is used to indicate if all msgs need to be written to the device */
#define TFA_MAX_MSGS                    (10)

#pragma pack (push, 1)

typedef struct uint24 {
  uint8_t b[3];
} uint24_t;
typedef struct nxpTfaHeader {
	uint16_t id;
    char version[2];     
    char subversion[2];  
    uint16_t size;       
    uint32_t CRC;        
    char customer[8];    
    char application[8]; 
    char type[8];		 
} nxpTfaHeader_t;

typedef enum nxpTfaSamplerate {
	fs_8k,       
	fs_11k025,   
	fs_12k,      
	fs_16k,      
	fs_22k05,    
	fs_24k,      
	fs_32k,      
	fs_44k1,     
	fs_48k,      
	fs_96k       
} nxpTfaSamplerate_t;

typedef struct nxpTfaDspMem {
	uint8_t  type;            
	uint16_t address;		
	uint8_t size; 				
	int words[]; 				
} nxpTfaDspMem_t;

#define BIQUAD_COEFF_SIZE       6

typedef struct {
	int a2;
	int a1;	
	int b2;	
	int b1;	
	int b0;	
}nxpTfaBiquad_t;

typedef struct nxpTfaBiquadOld {
  uint8_t bytes[BIQUAD_COEFF_SIZE*sizeof(uint24_t)];
}nxpTfaBiquadOld_t;

typedef struct nxpTfaBiquadFloat {
  float headroom;
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
} nxpTfaBiquadFloat_t;

typedef enum nxpTfaFilterType {
       fFlat,           
       fLowpass,        
       fHighpass,       
       fLowshelf,
       fHighshelf,
       fNotch,
       fPeak,
       fBandpass,
       f1stLP,
       f1stHP,
       fElliptic
} nxpTfaFilterType_t;

typedef struct nxpTfaFilter {
  nxpTfaBiquadOld_t biquad;
  uint8_t enabled;
  uint8_t type; 
  float frequency;
  float Q;
  float gain;
} nxpTfaFilter_t ;  


#define TFA_BQ_EQ_INDEX 0
#define TFA_BQ_ANTI_ALIAS_INDEX 10
#define TFA_BQ_INTEGRATOR_INDEX 13

typedef struct nxpTfaLsCompensationFilter {
  nxpTfaBiquad_t biquad;
  uint8_t lsCompOn;  
  uint8_t bwExtOn;   
  float fRes;        
  float Qt;          
  float fBwExt;      
  float samplingFreq;
} nxpTfaLsCompensationFilter_t;

typedef struct nxpTfaAntiAliasFilter {
  nxpTfaBiquad_t biquad;	
  uint8_t enabled;
  float cutOffFreq;   
  float samplingFreq; 
  float rippleDb;     
  float rolloff;      
} nxpTfaAntiAliasFilter_t;

typedef struct nxpTfaIntegratorFilter {
  nxpTfaBiquad_t biquad; 
  uint8_t type;             
  float  cutOffFreq;        
  float  samplingFreq;      
  float  leakage;           
} nxpTfaIntegratorFilter_t;


typedef struct nxpTfaEqFilter {
  nxpTfaBiquad_t biquad;
  uint8_t enabled;
  uint8_t type;       
  float cutOffFreq;   
  float samplingFreq; 
  float Q;            
  float gainDb;       
} nxpTfaEqFilter_t ;  

typedef struct nxpTfaContAntiAlias {
	int8_t index; 	
	uint8_t type;
	float cutOffFreq;   
	float samplingFreq;
	float rippleDb;     
	float rolloff;
	uint8_t bytes[5*3];	
}nxpTfaContAntiAlias_t;

typedef struct nxpTfaContIntegrator {
	int8_t index; 	
	uint8_t type;
	float cutOffFreq;   
	float samplingFreq;
	float leakage;     
	float reserved;
	uint8_t bytes[5*3];	
}nxpTfaContIntegrator_t;
typedef struct nxpTfaContEq {
  int8_t index;
  uint8_t type;       
  float cutOffFreq;   
  float samplingFreq; 
  float Q;            
  float gainDb;       
  uint8_t bytes[5*3];	
} nxpTfaContEq_t ;  

typedef union nxpTfaContBiquad {
	nxpTfaContEq_t eq;
	nxpTfaContAntiAlias_t aa;
	nxpTfaContIntegrator_t in;
}nxpTfaContBiquad_t;

#define TFA_BQ_EQ_INDEX 0
#define TFA_BQ_ANTI_ALIAS_INDEX 10
#define TFA_BQ_INTEGRATOR_INDEX 13

#define TFA98XX_MAX_EQ 10
typedef struct nxpTfaEqualizer {
  nxpTfaFilter_t filter[TFA98XX_MAX_EQ];
} nxpTfaEqualizer_t;

#define HDR(c1,c2) (c2<<8|c1) 
typedef enum nxpTfaHeaderType {
    paramsHdr		= HDR('P','M'), 
    volstepHdr	 	= HDR('V','P'),
    patchHdr	 	= HDR('P','A'),
    speakerHdr	 	= HDR('S','P'),
    presetHdr	 	= HDR('P','R'),
    configHdr	 	= HDR('C','O'),
    equalizerHdr	= HDR('E','Q'),
    drcHdr		= HDR('D','R'),
    msgHdr		= HDR('M','G'),	
    infoHdr		= HDR('I','N')
} nxpTfaHeaderType_t;

#define NXPTFA_EQ_VERSION    '1'
#define NXPTFA_EQ_SUBVERSION "00"
typedef struct nxpTfaEqualizerFile {
	nxpTfaHeader_t hdr;
	uint8_t samplerate; 				 
    nxpTfaFilter_t filter[TFA98XX_MAX_EQ];
} nxpTfaEqualizerFile_t;

#define NXPTFA_PA_VERSION    '1'
#define NXPTFA_PA_SUBVERSION "00"
typedef struct nxpTfaPatchFile {
	nxpTfaHeader_t hdr;
	uint8_t data[];
} nxpTfaPatch_t;

#define NXPTFA_MG_VERSION    '3'
#define NXPTFA_MG_SUBVERSION "00"
typedef struct nxpTfaMsgFile {
	nxpTfaHeader_t hdr;
	uint8_t data[];
} nxpTfaMsgFile_t;

#define NXPTFA_CO_VERSION    '1'
#define NXPTFA_CO3_VERSION   '3'
#define NXPTFA_CO_SUBVERSION1 "01"
typedef struct nxpTfaConfigS1File {
	nxpTfaHeader_t hdr;
	uint8_t data[55*3];
} nxpTfaConfigS1_t;
#define NXPTFA_CO_SUBVERSION2 "02"
typedef struct nxpTfaConfigS2File {
	nxpTfaHeader_t hdr;
	uint8_t data[67*3];
} nxpTfaConfigS2_t;
#define NXPTFA_CO_SUBVERSION3 "03"
typedef struct nxpTfaConfigS3File {
	nxpTfaHeader_t hdr;
	uint8_t data[67*3];
} nxpTfaConfigS3_t;

#define NXPTFA_CO_SUBVERSION "00"
typedef struct nxpTfaConfigFile {
	nxpTfaHeader_t hdr;
	uint8_t data[];
} nxpTfaConfig_t;

#define NXPTFA_PR_VERSION    '1'
#define NXPTFA_PR_SUBVERSION "00"

typedef struct nxpTfaPresetFile {
	nxpTfaHeader_t hdr;
	uint8_t data[];
} nxpTfaPreset_t;
#define NXPTFA_DR_VERSION    '1'
#define NXPTFA_DR_SUBVERSION "00"
typedef struct nxpTfaDrcFile {
	nxpTfaHeader_t hdr;
	uint8_t data[];
} nxpTfaDrc_t;

#define NXPTFA_DR3_VERSION    '3'
#define NXPTFA_DR3_SUBVERSION "00"
typedef struct nxpTfaDrcFile2 {
	nxpTfaHeader_t hdr;
	uint8_t version[3];
	uint8_t data[];
} nxpTfaDrc2_t;


#define NXPTFA_VP1_VERSION    '1'
#define NXPTFA_VP1_SUBVERSION "01"
typedef struct nxpTfaVolumeStep1 {
    float attenuation;              
    uint8_t preset[TFA98XX_PRESET_LENGTH];
} nxpTfaVolumeStep1_t;

#define NXPTFA_VP2_VERSION    '2'
#define NXPTFA_VP2_SUBVERSION "01"
typedef struct nxpTfaVolumeStep2 {
    float attenuation;              
    uint8_t preset[TFA98XX_PRESET_LENGTH];
    nxpTfaFilter_t filter[TFA98XX_MAX_EQ];
} nxpTfaVolumeStep2_t;


#define NXPTFA_VP_VERSION    '1'
#define NXPTFA_VP_SUBVERSION "00"
typedef struct nxpTfaVolumeStepFile {
	nxpTfaHeader_t hdr;
	uint8_t vsteps;  	
	uint8_t samplerate; 
	uint8_t payload; 	
}nxpTfaVolumeStepFile_t;
typedef struct nxpTfaVolumeStep2File {
	nxpTfaHeader_t hdr;
	uint8_t vsteps;  	
	uint8_t samplerate; 
	nxpTfaVolumeStep2_t vstep[]; 	
}nxpTfaVolumeStep2File_t;

typedef struct nxpTfaVolumeStepMax2File {
	nxpTfaHeader_t hdr;
	uint8_t version[3]; 
	uint8_t NrOfVsteps;
	uint8_t vstepsBin[]; 
}nxpTfaVolumeStepMax2File_t;

typedef struct nxpTfaVolumeStepMax2_1File {
	uint8_t version[3]; 
	uint8_t NrOfVsteps;
	uint8_t vstepsBin[]; 
}nxpTfaVolumeStepMax2_1File_t;

struct nxpTfaVolumeStepRegisterInfo {
	uint8_t NrOfRegisters;
	uint16_t registerInfo[];
};

struct nxpTfaVolumeStepMessageInfo {
	uint8_t NrOfMessages;
	uint8_t MessageType;
	uint24_t MessageLength;
	uint8_t CmdId[3];
	uint8_t ParameterData[];
};

typedef struct nxpTfaOldHeader {
	uint16_t id;
	char version[2];     
	char subversion[2];  
	uint16_t size;       
	uint32_t CRC;        
} nxpTfaOldHeader_t;

typedef struct nxpOldTfaFilter {
  double bq[5];
  int32_t type;
  double frequency;
  double Q;
  double gain;
  uint8_t enabled;
} nxpTfaOldFilter_t ;  
typedef struct nxpTfaOldVolumeStep2 {
    float attenuation;              
    uint8_t preset[TFA98XX_PRESET_LENGTH];
    nxpTfaOldFilter_t eq[10];
} nxpTfaOldVolumeStep2_t;
typedef struct nxpTfaOldVolumeStepFile {
	nxpTfaOldHeader_t hdr;
	nxpTfaOldVolumeStep2_t step[];

}nxpTfaOldVolumeStep2File_t;

struct nxpTfaSpkHeader {
	struct nxpTfaHeader hdr;
	char name[8];				
	char vendor[16];
	char type[8];
	
	uint8_t height;
	uint8_t width;
	uint8_t depth;
	uint16_t ohm;
};


#define NXPTFA_SP_VERSION    '1'
#define NXPTFA_SP_SUBVERSION "00"
typedef struct nxpTfaSpeakerFile {
	nxpTfaHeader_t hdr;
	char name[8];				
	char vendor[16];
	char type[8];
	
	uint8_t height;
	uint8_t width;
	uint8_t depth;
	uint8_t ohm_primary;
	uint8_t ohm_secondary;
	uint8_t data[]; 
} nxpTfaSpeakerFile_t;

#if ( defined( TFA9888 ) || defined( TFA98XX_FULL ))

#define NXPTFA_VP3_VERSION    '3'
#define NXPTFA_VP3_SUBVERSION "00"

struct nxpTfaFWVer {
	uint8_t Major;
	uint8_t minor;
	uint8_t minor_update:6;
	uint8_t Update:2;
}; 

struct nxpTfaCmdID {
	int a;
	
	
	
}; 

struct nxpTfaFWMsg {
	struct nxpTfaFWVer fwVersion;
	struct nxpTfaMsg payload;
};

typedef struct nxpTfaLiveData {
	uint8_t liveData_size;
        char name[25];
	char addrs[25];
        int tracker;
	int scalefactor[MEMTRACK_MAX_WORDS];
} nxpTfaLiveData_t;

#define NXPTFA_SP3_VERSION  '3'
#define NXPTFA_SP3_SUBVERSION "00"
struct nxpTfaSpeakerFileMax2 {
	nxpTfaHeader_t hdr;
	char name[8];				
	char vendor[16];
	char type[8];
	
	uint8_t height;
	uint8_t width;
	uint8_t depth;
	uint8_t ohm_primary;
	uint8_t ohm_secondary;
	struct nxpTfaFWMsg FWmsg; 
};
#endif

typedef enum nxpTfaDescriptorType {
	dscDevice,		
	dscProfile,		
	dscRegister,	        
	dscString,		
	dscFile,		
	dscPatch,               
	dscMarker,		
	dscMode,
#if 1 
	dscSetInputSelect,
	dscSetOutputSelect,
	dscSetProgramConfig,
	dscSetLagW,
	dscSetGains,
	dscSetvBatFactors,
	dscSetSensesCal,
	dscSetSensesDelay,
#endif
	dscBitfield,
	dscDefault,             
	dscLiveData,
	dscLiveDataString,
	dscGroup,
	dscCmd,
#if 1 
	dscSetMBDrc,
#endif
	dscFilter,
	dscNoInit,
        dscFeatures,
	dscCfMem, 
	dsc_last	
} nxpTfaDescriptorType_t;

#define TFA_BITFIELDDSCMSK 0x7fffffff

typedef struct nxpTfaDescPtr {
	uint32_t offset:24;
	uint32_t  type:8; 
}nxpTfaDescPtr_t;

typedef struct nxpTfaFileDsc {
	nxpTfaDescPtr_t name;
	uint32_t size;	
	uint8_t data[]; 
} nxpTfaFileDsc_t;


typedef struct nxpTfaDeviceList {
	uint8_t length;			
	uint8_t bus;			
	uint8_t dev;			
	uint8_t func;			
	uint32_t devid;		        
	nxpTfaDescPtr_t name;	        
	nxpTfaDescPtr_t list[];	        
} nxpTfaDeviceList_t;

typedef struct nxpTfaProfileList {
	uint32_t length:8;		
	uint32_t group:8;		
	uint32_t ID:16;			
	nxpTfaDescPtr_t name;	        
	nxpTfaDescPtr_t list[];	        
} nxpTfaProfileList_t;
#define TFA_PROFID 0x1234

typedef struct nxpTfaLiveDataList {
	uint32_t length:8;		
	uint32_t ID:24;			
	nxpTfaDescPtr_t name;	        
	nxpTfaDescPtr_t list[];	        
} nxpTfaLiveDataList_t;
#define TFA_LIVEDATAID 0x5678

typedef struct nxpTfaBitfield {
	uint16_t  value;
	uint16_t  field; 
} nxpTfaBitfield_t;

typedef struct nxpTfaBfEnum {
	unsigned int  len:4;		
	unsigned int  pos:4;
	unsigned int  address:8;
} nxpTfaBfEnum_t;

typedef struct nxpTfaRegpatch {
	uint8_t   address;	
	uint16_t  value;	
	uint16_t  mask;		
} nxpTfaRegpatch_t;

typedef struct nxpTfaUseCase {
	int value;	
} nxpTfaMode_t;

typedef struct nxpTfaNoInit {
	uint8_t value;	
} nxpTfaNoInit_t;

typedef struct nxpTfaFeatures {
	uint16_t value[3];	
} nxpTfaFeatures_t;

typedef struct nxpTfaCmd {
	uint16_t id;
	unsigned char value[3];
} nxpTfaCmd_t;

#define NXPTFA_PM_VERSION  '1'
#define NXPTFA_PM3_VERSION '3'
#define NXPTFA_PM_SUBVERSION '1'
typedef struct nxpTfaContainer {
    char id[2];          
    char version[2];     
    char subversion[2];  
    uint32_t size;       
    uint32_t CRC;        
    uint16_t rev;		 
    char customer[8];    
    char application[8]; 
    char type[8];		 
    uint16_t ndev;	 	 
    uint16_t nprof;	 	 
    uint16_t nliveData;          
    nxpTfaDescPtr_t index[]; 
} nxpTfaContainer_t;


#pragma pack (pop)



#endif 

/**
*	@file
*	@brief	計測ライブラリー							Ver 1.0.9.x
*/
/*============================================================================*/
#ifndef MEASUREMENT_LIBRARY_H_
#define MEASUREMENT_LIBRARY_H_


/*----------------------------------------------------------------------*/
/**
*	@brief	Mixing coefficient（mlCalMixCoef関数）用の入力値
*/
struct tagMlMixingValue
{
	double	radianX;
	double	radianY;

	double	hx45x;
	double	hy45x;
	double	hy45y;
	double	hx45y;

	UINT8	hxsx;
	UINT8	hysx;

	INT32	hx45xL;		//! for Fixed point
	INT32	hy45xL;		//! for Fixed point
	INT32	hy45yL;		//! for Fixed point
	INT32	hx45yL;		//! for Fixed point
};
/**
*	@brief	Mixing coefficient（mlCalMixCoef関数）用の入力値
*/
typedef	struct tagMlMixingValue		mlMixingValue;

/*----------------------------------------------------------------------*/
/**
*	@brief	Lineaity correction（mlCalLinearCorr関数）用の入力値
*/
struct tagMlLinearityValue
{
	int		measurecount;			//! input parameter
	UINT32	*dacX;			//! input parameter
	UINT32	*dacY;			//! input parameter

	double	*positionX;
	double	*positionY;
	UINT16	*thresholdX;		
	UINT16	*thresholdY;

	UINT32	*coefAXL;		//! for Fixed point
	UINT32	*coefBXL;		//! for Fixed point
	UINT32	*coefAYL;		//! for Fixed point
	UINT32	*coefBYL;		//! for Fixed point
};
/**
*	@brief	Linearity correction（mlCalLinearCorr関数）用の入力値
*/
typedef	struct tagMlLinearityValue		mlLinearityValue;

#endif /* #ifndef MEASUREMENT_LIBRARY_H_ */

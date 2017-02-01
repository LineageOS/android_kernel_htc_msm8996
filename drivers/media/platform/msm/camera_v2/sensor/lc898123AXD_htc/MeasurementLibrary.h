/**
*	@file
*	@brief	�v�����C�u�����[							Ver 1.0.9.x
*/
/*============================================================================*/
#ifndef MEASUREMENT_LIBRARY_H_
#define MEASUREMENT_LIBRARY_H_


/*----------------------------------------------------------------------*/
/**
*	@brief	Mixing coefficient�imlCalMixCoef�֐��j�p�̓��͒l
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
*	@brief	Mixing coefficient�imlCalMixCoef�֐��j�p�̓��͒l
*/
typedef	struct tagMlMixingValue		mlMixingValue;

/*----------------------------------------------------------------------*/
/**
*	@brief	Lineaity correction�imlCalLinearCorr�֐��j�p�̓��͒l
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
*	@brief	Linearity correction�imlCalLinearCorr�֐��j�p�̓��͒l
*/
typedef	struct tagMlLinearityValue		mlLinearityValue;

#endif /* #ifndef MEASUREMENT_LIBRARY_H_ */

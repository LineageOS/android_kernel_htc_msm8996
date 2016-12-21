#ifndef MEASUREMENT_LIBRARY_H_
#define MEASUREMENT_LIBRARY_H_


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

	INT32	hx45xL;		
	INT32	hy45xL;		
	INT32	hy45yL;		
	INT32	hx45yL;		
};
typedef	struct tagMlMixingValue		mlMixingValue;

struct tagMlLinearityValue
{
	int		measurecount;			
	UINT32	*dacX;			
	UINT32	*dacY;			

	double	*positionX;
	double	*positionY;
	UINT16	*thresholdX;		
	UINT16	*thresholdY;

	UINT32	*coefAXL;		
	UINT32	*coefBXL;		
	UINT32	*coefAYL;		
	UINT32	*coefBYL;		
};
typedef	struct tagMlLinearityValue		mlLinearityValue;

#endif 

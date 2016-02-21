#include "stdafx.h"
#include "correspondence.h"


correspondence::correspondence()
{
	x1 = 0;
	y1 = 0;
	z1 = 0;
	x2 = 0;
	y2 = 0;
	z2 = 0;
	distance = 0;
}

correspondence::correspondence(double tx1, double ty1, double tz1, double tx2, double ty2, double tz2, double tdistance)
{
	x1 = tx1;
	y1 = ty1;
	z1 = tz1;

	x2 = tx2;
	y2 = ty2;
	z2 = tz2;
	distance = tdistance;
}

correspondence::~correspondence()
{
}

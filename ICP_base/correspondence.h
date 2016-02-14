#pragma once
class correspondence
{
public:
	correspondence();
	correspondence(double tx1, double ty1, double tz1,
		double tx2, double ty2, double tz2,
		double tdistance);
	~correspondence();
	double x1, y1, z1;
	double x2, y2, z2;
	double distance;

	
};


#pragma once
#include <pcl/point_types.h>

namespace pcl
{
	struct LFSHSignature
	{
		//N1 local depth:10	 0-9
		//N2 deviation angle:15	  10-24
		//N3 density :5	   25-29
		float histogram[30];

		int getNumberOfDimensions() const;

		LFSHSignature();		
	};

	inline int LFSHSignature::getNumberOfDimensions() const
	{
		return 30;
	}

	inline LFSHSignature::LFSHSignature()
	{
		for (int i(0); i < 30; ++i)
		{
			histogram[i] = 0.0f;
		}
	}
}


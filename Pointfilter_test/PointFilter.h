#pragma once

#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

template<typename PointSource>
class PointFilter
{
public:
	PointFilter();

	~PointFilter();
	

	bool setInputSource( pcl::PointCloud<PointSource>& input_cloud);
	bool computMassCenter();

private:
	typename pcl::PointCloud<PointSource>::Ptr _input_cloud_ptr_;

};


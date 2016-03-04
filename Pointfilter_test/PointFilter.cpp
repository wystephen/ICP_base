#include "PointFilter.h"


template <typename PointSource>
PointFilter<PointSource>::PointFilter()
{
}

template <typename PointSource>
PointFilter<PointSource>::~PointFilter()
{
}

template <typename PointSource>
bool PointFilter<PointSource>::setInputSource( pcl::PointCloud<PointSource>& input_cloud)
{
	*_input_cloud_ptr_ = input_cloud;
	if (input_cloud->size() == 0)
		return false;
	else
		return true;
}

template <typename PointSource>
bool PointFilter<PointSource>::computMassCenter()
{
	int cloud_size = _input_cloud_ptr_->size();
	std::cout <<"cloud size:"<< cloud_size << std::endl;
	int sum_x(0), sum_y(0), sum_z(0);
	for (int i(0); i<cloud_size; i++)
	{
		sum_x += _input_cloud_ptr_->at(i).x;
		sum_y += _input_cloud_ptr_->at(i).y;
		sum_z += _input_cloud_ptr_->at(i),z;
		
	}
	sum_x = sum_x / cloud_size;
	sum_y = sum_y / cloud_size;
	sum_z = sum_z / cloud_size;


	return true;


}

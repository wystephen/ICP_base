
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl\registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "PointFilter.h"


int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *pc);

	pcl::PointCloud<pcl::PointXYZ>::Ptr end(new pcl::PointCloud<pcl::PointXYZ>);

	PointFilter<pcl::PointXYZ> point_filter;
	point_filter.setInputSource(*pc);
	point_filter.computMassCenter();


	Sleep(10000);



	return 0;
}

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl\registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>



int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr p_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture1_B.pcd", *p_src_ptr);
	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture2_B.pcd", *p_target_ptr);

	pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *pc);





	Sleep(10000);



	return 0;
}
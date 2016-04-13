#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl\registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/histogram_visualizer.h>


#include "LFSHSignature.h"

#include <pcl/filters/approximate_voxel_grid.h>

#include "LFSH.h"
#include "OSAC.h"

#include "kdtree_feature.h"

#include "RANSAC.h"

#define COMPRESS_SIZE 0.051

bool PointCloudTransform(pcl::PointCloud<pcl::PointXYZ>& src, pcl::PointCloud<pcl::PointXYZ> & target,
	Eigen::MatrixXf transform_matrix)
{
	target.clear();
	for (int i(0); i < src.size();++i)
	{
		Eigen::Vector4f tmp;
		tmp(0) = src.at(i).x;
		tmp(1) = src.at(i).y;
		tmp(2) = src.at(i).z;
		tmp(3) = 1.0;
		tmp = tmp * transform_matrix;
		pcl::PointXYZ p;
		p.x = tmp(0);
		p.y = tmp(1);
		p.z = tmp(2);
		target.push_back(p);
	}
	return true;
}

int main()
{

	/********************************************************************************************* /
	//Model test for LFSH

	//ModelTest_LFSH();   // pase?

	//ModelTest_LM6DOF();	 // pase

	pcl::OSAC<pcl::PointXYZ, pcl::LFSHSignature> osac_test;

	osac_test.setCompressSize(0.02);
	osac_test.setNumberOfCorr(3);
	osac_test.setAlpha(1.5);

	//osac_test.ModelTest_OSAC();



	/*********************************************************************************/
	/***************************************************************
	初始化可视化工具
	
	*************************************************/
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viwer_ptr(new pcl::visualization::PCLVisualizer);
	viwer_ptr->setBackgroundColor(0, 0, 0);

	pcl::LFSH<pcl::PointXYZ, pcl::LFSHSignature> lfsh_extraction;


	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr p_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr p_src_small_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_target_small_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::LFSHSignature>::Ptr p_src_lpfh_ptr(new pcl::PointCloud<pcl::LFSHSignature>);
	pcl::PointCloud<pcl::LFSHSignature>::Ptr p_target_lpfh_ptr(new pcl::PointCloud<pcl::LFSHSignature>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr p_icp_ptr(new pcl::PointCloud<pcl::PointXYZ>);


	//pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *p_src_ptr);
	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture5_B.pcd", *p_src_ptr);

	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture1_B.pcd", *p_target_ptr);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *pc);



	//std::cout << "source size:" << p_src_ptr->size() << std::endl;
	/************************************************************* /
	//这一部分代码用于测试LPFH类。
	pcl::ApproximateVoxelGrid<pcl::PointXYZ>::Ptr compress_ptr(new pcl::ApproximateVoxelGrid<pcl::PointXYZ>);
	compress_ptr->setInputCloud(p_src_ptr);
	compress_ptr->setDownsampleAllData(true);
	compress_ptr->setLeafSize(COMPRESS_SIZE, COMPRESS_SIZE, COMPRESS_SIZE);
	//compress_ptr->filter(*p_src_small_ptr);

	compress_ptr->setInputCloud(p_target_ptr);		  
	compress_ptr->filter(*p_src_small_ptr);			  

	lfsh_extraction.setInputCloud(p_src_small_ptr);
	lfsh_extraction.compute(*p_src_lpfh_ptr);

	pcl::kdtree_feature<pcl::LFSHSignature> kd_featrure;
	kd_featrure.setInputCloud(p_src_lpfh_ptr);

	std::vector<int> idx_vector;
	std::vector<float> distance_vector;

	kd_featrure.searchNNearest(p_src_lpfh_ptr->at(3), 20, idx_vector, distance_vector);
	for (int i(0); i < idx_vector.size();++i)
	{
		std::cout << i << ":" << idx_vector[i] <<"-"<<distance_vector[i]<< std::endl;
	}

	std::cout << "p_src_lpfh size:" << p_src_lpfh_ptr->size() << std::endl;


	std::cout << "Is true:" << lfsh_extraction.ForDebugInputViwer(viwer_ptr) << std::endl;;
	lfsh_extraction.DisplayInput();
	/***************************************************************/


	/**************************************/
	//这一部分用来测试OSAC类
	//viwer_ptr->spin();
	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

	double theta = M_PI / 4.0/1.0;

	transform_matrix(0, 0) = cos(theta);
	transform_matrix(0, 1) = -sin(theta);
	transform_matrix(1, 0) = sin(theta);
	transform_matrix(1, 1) = cos(theta);
	
	transform_matrix(0, 3) = 4;
	transform_matrix(1, 3) = 4;

	


	//pcl::transformPointCloud(*p_src_ptr, *p_target_ptr, transform_matrix);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> p_src_transform(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	//pcl::RANSAC<pcl::PointXYZ, pcl::LFSHSignature> ransac;

	viwer_ptr->addCoordinateSystem(1.0);
	for (int i(0); i < 30001; i++)
	{
		pcl::OSAC<pcl::PointXYZ, pcl::LFSHSignature> osac_estimation;
		viwer_ptr->removeAllPointClouds();
		
		osac_estimation.setCompressSize(0.08);
		osac_estimation.setNumberOfCorr(3);
		osac_estimation.setAlpha(1.5);

		osac_estimation.setSourceCloud(p_src_ptr);
		osac_estimation.setTargetCloud(p_target_ptr);

		boost::shared_ptr<Eigen::Matrix4f>
			transform_matrix_ptr(new Eigen::Matrix4f);
		osac_estimation.compute(*transform_matrix_ptr);	 

		pcl::transformPointCloud(*p_src_ptr, *p_src_transform, *transform_matrix_ptr);

		icp.setInputSource(p_src_transform);
		icp.setInputTarget(p_target_ptr);
		icp.align(*p_icp_ptr);
		std::cout << "icp matrix:" << std::endl;

		std::cout << icp.getFinalTransformation() << std::endl;

		//pcl::transformPointCloud(*p_target_ptr, *p_src_transform, *transform_matrix_ptr);
		//PointCloudTransform(*p_src_ptr, *p_src_transform, *transform_matrix_ptr);	
		//红色：目标     绿色：转换后         蓝色：原始	other:icp 										   
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_src(p_src_ptr, 2, 2, 222);
		viwer_ptr->addPointCloud(p_src_ptr, single_color_src, "src");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_transform(p_src_transform, 0, 222, 22);
		viwer_ptr->addPointCloud(p_src_transform, single_color_transform, "transform");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_target(p_target_ptr, 222, 2, 2);
		viwer_ptr->addPointCloud(p_target_ptr, single_color_target,"target");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_icp(p_icp_ptr, 100, 100, 100);
		viwer_ptr->addPointCloud(p_icp_ptr, single_color_icp, "icp");

		std::cout << "-------------------------------------------------------------------------------------------" << std::endl;

		viwer_ptr->spinOnce(1000);
		//Sleep(4);
		//int a(0);
		//cin >> a;
		
	}


	/************************************************************/
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_1(p_src_ptr, 0, 255, 0);
	//viwer_ptr->addPointCloud(p_src_ptr,single_color_1,"src");


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_2(p_target_ptr, 244, 2, 2);
	//viwer_ptr->addPointCloud(p_target_ptr, single_color_2, "target");

	/********************************************************* /
	pcl::visualization::PCLHistogramVisualizer hist_visualizer;
	hist_visualizer.addFeatureHistogram<pcl::LFSHSignature>(*p_src_lpfh_ptr, 30,"src");
	hist_visualizer.spin();
	/**********************************************************/


	while (! viwer_ptr->wasStopped())
	{
		viwer_ptr->spin();
	}

	return 0;
}


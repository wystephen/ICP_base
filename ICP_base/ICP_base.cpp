// ICP_base.cpp : 定义控制台应用程序的入口点。
//
/******************************************************************************
   
*******************************************************************************/


#include "stdafx.h"



#include "ICP.h"
#include "ICP.cpp"
//#include "FUNCTOR.h"
//#include "FUNCTOR.cpp"
//#include "TransfromEstimationLM.h"
//#include "TransfromEstimationLM.cpp"


#include <math.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>

int _tmain(int argc, _TCHAR* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	//pc  = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>> ();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_t(new pcl::PointCloud<pcl::PointXYZ>);
	//pc_t =(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	//other pcd
	pcl::PointCloud<pcl::PointXYZ> ::Ptr pcb1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcb2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcb3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcb4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcb5(new pcl::PointCloud<pcl::PointXYZ>);
	/*               * /
	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture1_B.pcd",*pcb1);
	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture2_B.pcd", *pcb2);
	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture4_B.pcd", *pcb3);
	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture4_B.pcd", *pcb4);
	pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture5_B.pcd", *pcb5);
	**********************************************************************/

	
	pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *pc);


	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

	double theta =  M_PI /4.0;

	transform_matrix(0, 0) = cos(theta);
	transform_matrix(0, 1) = -sin(theta);
	transform_matrix(1, 0) = sin(theta);
	transform_matrix(1, 1) = cos(theta);

	transform_matrix(0, 3) = 0.1;

	pcl::transformPointCloud(*pc, *pc_t, transform_matrix);
	//before here,loaded pcd file,transfrom to anther one.

	pcl::PointCloud<pcl::PointXYZ>  Final;
	//pcl icp achive 
	/* *******************************/
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;				
	icp.setInputSource(pc);
	icp.setInputTarget(pc_t);
	
	*Final_ptr = Final;
	

	icp.align(Final);
	std::cout << "source:" << transform_matrix << std::endl;
	std::cout <<"ICP Final:"<< icp.getFinalTransformation() << std::endl;
	  /*******************************************/
	ICP<pcl::PointXYZ, pcl::PointXYZ> o_icp;
	o_icp.setSource(pc);
	o_icp.setTarget(pc_t);

	std::cout << "begin oicp cptran func" << std::endl;
	o_icp.computerTransformation(Final, icp.getFinalTransformation());
	//o_icp.computerTransformation(Final, transform_matrix);
	//o_icp.computerTransformation(Final, Eigen::Matrix4f::Identity());





	//visualization code. do not change.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewers_ptr(new pcl::visualization::PCLVisualizer("office chair model"));


	viewers_ptr->setBackgroundColor(0, 0, 0);

	//viewers_ptr->addPointCloud<pcl::PointXYZ>(Final_ptr, "Final");			//ToDo:test
	//viewers_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Final");


	viewers_ptr->addPointCloud<pcl::PointXYZ>(pc, "sample");
	
	viewers_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample");

	viewers_ptr->addPointCloud<pcl::PointXYZ>(pc_t, "sa");
	viewers_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sa");


	viewers_ptr->initCameraParameters();


	while (!viewers_ptr->wasStopped())
	{
		viewers_ptr->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

	return 0;
}


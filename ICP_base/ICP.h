#pragma once

#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/common.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "correspondence.h"
#include "FUNCTOR.h"
#include "FUNCTOR.cpp"
#include "TransfromEstimationLM.h"
#include "TransfromEstimationLM.cpp"


#define threshold_distance_squre  1.0

template <typename PointSource,typename PointTarget>
class ICP
{
public:

	//ypedef boost::shared_ptr<ICP<PointSource, PointTarget>>  Ptr;
	//typedef boost::shared_ptr<const ICP<PointSource, PointTarget>>  ConstPtr;

	typedef boost::shared_ptr<pcl::PointCloud<PointSource>> PointCloudPtr;
	typedef const boost::shared_ptr<const pcl::PointCloud<PointSource>> PointCloudConstPtr;
																						  
	void setSource( const PointCloudConstPtr &in);
	void setTarget( const PointCloudConstPtr &target);

	void computerTransformation(pcl::PointCloud<PointSource>& output, const Eigen::Matrix4f& guess);

	void transfromPointCloud(typename pcl::PointCloud<PointSource>::Ptr input,
		pcl::PointCloud<PointSource> &output,
		const Eigen::Matrix4f &transform_matrix);

	void simpleFindCorrespondence(typename  pcl::PointCloud<PointSource>::Ptr input);


	void getTransLM();
	

	ICP():source_ptr_(new pcl::PointCloud<PointSource>),target_ptr_(new pcl::PointCloud<PointSource>),
		kdtree_target_ptr_(new pcl::KdTreeFLANN<PointSource>)
	{
		nr_iterations_ = 0;
	converged_ = false;
	//typename pcl::PointCloud<PointSource>::Ptr source_ptr_(new pcl::PointCloud<PointSource>);
	//typename pcl::PointCloud<PointSource>::Ptr target_ptr_(new pcl::PointCloud<PointSource>);
	//typename pcl::KdTreeFLANN<PointSource>::Ptr kdtree_target_ptr_(new pcl::KdTreeFLANN<PointSource>);

	final_transfrom_matrix = Eigen::Matrix4f::Identity();
	};
	~ICP();

private:
	Eigen::Matrix4f transform_matrix;
	Eigen::Matrix4f final_transfrom_matrix;

	 typename pcl::PointCloud<PointSource>::Ptr source_ptr_;
	 typename pcl::PointCloud<PointTarget>::Ptr target_ptr_;

	typename pcl::KdTreeFLANN<PointSource>::Ptr kdtree_target_ptr_;

	std::vector<correspondence> corr_vector_;



	int nr_iterations_;
	bool converged_;
	
	
};





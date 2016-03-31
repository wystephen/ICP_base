#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include "LFSHSignature.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>



namespace pcl
{
	template <typename PointInT, typename PointOutT>
	class LFSH
	{
	public:

		typedef boost::shared_ptr<pcl::PointCloud<PointInT>> PointCloudPtr;
		typedef const boost::shared_ptr<pcl::PointCloud<PointInT>> PointCloudConstPtr;

		typedef boost::shared_ptr<pcl::PointCloud<PointOutT>> PointLPFHPtr;

		typedef boost::shared_ptr<pcl::search::KdTree<PointInT>> KDTreePtr;

		LFSH() : model_volume_(0),
		         kdtree_ptr_(new pcl::search::KdTree<PointInT>),
		         input_ptr_(new pcl::PointCloud<PointInT>),
		         output_ptr_(new pcl::PointCloud<pcl::LFSHSignature>),
		         normal_ptr_(new pcl::PointCloud<pcl::Normal>)

		{
			r_ = 0.1;
			N1_ = 10;
			N2_ = 15;//15
			N3_ = 5;
		}

		~LFSH()
		{
		}

		/** \brief Display the input_ point cloud
		*/
		bool DisplayInput();

		/** \brief  **/
		bool ForDebugInputViwer(boost::shared_ptr<pcl::visualization::PCLVisualizer> view_shared_ptr);
		bool setInputCloud(const PointCloudPtr input);

		double setRadius_coeff(double coeff);

		bool compute(pcl::PointCloud<PointOutT>& output);

		double getModelSize()
		{
			return pow(model_volume_ / 3.1415926, 0.333333);
		}

	private:
		static double dot(Eigen::Vector3f x, Eigen::Vector3f y);

		int computeLocalDepth(Eigen::Vector3f source_normal,
		                      Eigen::Vector3f source_point,
		                      Eigen::Vector3f target_point) const;


		int computeDeviationAngle(Eigen::Vector3f source_normal,
		                          Eigen::Vector3f target_normal) const;

		int computeDensity(Eigen::Vector3f source_normal,
		                   Eigen::Vector3f source_point,
		                   Eigen::Vector3f target_point) const;

	protected:

		/** \brief The volume of model. **/
		double model_volume_;
		/** \brief The search radius. **/
		double r_;
		/** \brief The local depth feature's bin size. **/
		int N1_;
		/** \brief The deviation angle between normals feature's bin size. **/
		int N2_;
		/** \brief The point density feature's bin size. **/
		int N3_;

		/** \brief kdtree struct。**/
		KDTreePtr kdtree_ptr_;

		/** \brief Normal PointCloud Ptr. **/
		boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normal_ptr_;
		/** \brief Ptr save input point cloud. **/
		PointCloudPtr input_ptr_;

		/** \brief Ptr for output LFSH point cloud **/
		PointLPFHPtr output_ptr_;

		/** \brief The viewer point to debug,and visualization.**/
		boost::shared_ptr<pcl::visualization::PCLVisualizer> view_shared_ptr_;
	};

	template <typename PointInT, typename PointOutT>
	bool LFSH<PointInT, PointOutT>::ForDebugInputViwer(boost::shared_ptr<pcl::visualization::PCLVisualizer> view_shared_ptr)
	{
		if (!view_shared_ptr) return false;
		view_shared_ptr_ = view_shared_ptr;
		return true;
	}

	template <typename PointInT, typename PointOutT>
	bool LFSH<PointInT, PointOutT>::setInputCloud(PointCloudPtr input)
	{
		if (!input) return false;
		input_ptr_ = input;

		double model_x_max(-10000.0), model_y_max(-100000.0), model_z_max(-100000.0);
		double model_x_min(10000.0), model_y_min(1000000.0), model_z_min(1000000.0);

		double model_size(0.0);
		for (int i(0); i < input->size(); ++i)
		{
			//input_ptr_->push_back(input->at(i));
			if (input_ptr_->at(i).x > model_x_max)
			{
				model_x_max = input_ptr_->at(i).x;
			}
			if (input_ptr_->at(i).x < model_x_min)
			{
				model_x_min = input_ptr_->at(i).x;
			}

			if (input_ptr_->at(i).y > model_y_max)
			{
				model_y_max = input_ptr_->at(i).y;
			}
			if (input_ptr_->at(i).y < model_y_min)
			{
				model_y_min = input_ptr_->at(i).y;
			}

			if (input_ptr_->at(i).z > model_z_max)
			{
				model_z_max = input_ptr_->at(i).z;
			}
			if (input_ptr_->at(i).z < model_z_min)
			{
				model_z_min = input_ptr_->at(i).z;
			}
		}
		model_volume_ = (model_x_max - model_x_min) *
			(model_y_max - model_y_min) *
			(model_z_max - model_z_min);

		setRadius_coeff(0.25);

		return true;
	}

	template <typename PointInT, typename PointOutT>
	double LFSH<PointInT, PointOutT>::setRadius_coeff(double coeff)
	{
		r_ = pow(model_volume_ / M_PI * 3.0 / 4.0, 0.33333) * coeff;
		//std::cout << "r_:" << r_ << "model_volum:"<<model_volume_<<std::endl;
		return r_;
	}

	template <typename PointInT, typename PointOutT>
	bool LFSH<PointInT, PointOutT>::compute(pcl::PointCloud<PointOutT>& output)
	{
		//For debug:
#ifdef _DEBUG
		int index_1_max(0), index_1_min(33);
		int index_2_max(0), index_2_min(33);
		int index_3_max(0), index_3_min(33);
#endif
		//TODO: 这里有个问题。//解决了，注意初始化指针。

		kdtree_ptr_->setInputCloud(input_ptr_);

		boost::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>> tmp_kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);

		pcl::NormalEstimationOMP<PointInT, pcl::Normal> ne;
		ne.setInputCloud(input_ptr_);
		ne.setSearchMethod(tmp_kd_tree);
		ne.setRadiusSearch(r_ * 0.5);
		ne.setNumberOfThreads(4);
		ne.compute(*normal_ptr_);
		//TODO: 可以尝试实现用高密度点云估算法向量方向 setSearchSurface.

		std::vector<int> pointIdx_vector;
		std::vector<float> pointDistance_vector;

		//vector 点乘 x.dot(y)

		std::cout << "input size:" << input_ptr_->size() << std::endl;
		int i(0);

		for (i = 0; i < (input_ptr_->size()); ++i)
		{
			//std::cout << "i:" << i << std::endl;

			if (kdtree_ptr_->radiusSearch(input_ptr_->at(i), r_, pointIdx_vector, pointDistance_vector) > 0)
			{
				pcl::LFSHSignature tmp_signature;


				Eigen::Vector3f the_point_f;
				the_point_f(0) = input_ptr_->at(i).x;
				the_point_f(1) = input_ptr_->at(i).y;
				the_point_f(2) = input_ptr_->at(i).z;

				Eigen::Vector3f tmp_point;//q_i^j in paper
				Eigen::Vector3f tmp_source_normal;//n_i in paper
				Eigen::Vector3f tmp_target_normal;//n_i^j in paper

				tmp_source_normal = Eigen::Vector3f(
					normal_ptr_->at(i).normal_x,
					normal_ptr_->at(i).normal_y,
					normal_ptr_->at(i).normal_z);


				for (int j(0); j < pointIdx_vector.size(); ++j)
				{
					tmp_point(0) = input_ptr_->at(pointIdx_vector.at(j)).x;
					tmp_point(1) = input_ptr_->at(pointIdx_vector.at(j)).y;
					tmp_point(2) = input_ptr_->at(pointIdx_vector.at(j)).z;

					tmp_target_normal = Eigen::Vector3f(
						normal_ptr_->at(pointIdx_vector[j]).normal_x,
						normal_ptr_->at(pointIdx_vector[j]).normal_y,
						normal_ptr_->at(pointIdx_vector[j]).normal_z);

					int tmp_index_1(0);
					int tmp_index_2(0);
					int tmp_index_3(0);

					tmp_index_1 = computeLocalDepth(tmp_source_normal, the_point_f, tmp_point);
#ifdef _DEBUG
					if (tmp_index_1 > index_1_max) index_1_max = tmp_index_1;
					if (tmp_index_1 < index_1_min) index_1_min = tmp_index_1;
#endif
					tmp_signature.histogram[tmp_index_1] += (1.0f);

					tmp_index_2 = N1_ + computeDeviationAngle(tmp_source_normal, tmp_target_normal);
#ifdef _DEBUG
					if (tmp_index_2 > index_2_max) index_2_max = tmp_index_2;
					if (tmp_index_2 < index_2_min) index_2_min = tmp_index_2;
#endif
					tmp_signature.histogram[tmp_index_2] += (1.0f);


					tmp_index_3 = N1_ + N2_ + computeDensity(tmp_source_normal, the_point_f, tmp_point);
#ifdef _DEBUG
					if (tmp_index_3 > index_3_max) index_3_max = tmp_index_3;
					if (tmp_index_3 < index_3_min) index_3_min = tmp_index_3;
#endif
					tmp_signature.histogram[tmp_index_3] += (1.0f);
				}
				/****************************************************/
				double test_sum(0.0);

				for (int k(0); k < 30; ++k)
				{
					tmp_signature.histogram[k] = tmp_signature.histogram[k] / double(pointIdx_vector.size());
					test_sum += tmp_signature.histogram[k];
					//std::cout << ":" << tmp_signature.histogram[k];
				}
				//std::cout << std::endl;
				//std::cout << "test_sum:" << test_sum<<std::endl;
				//TODO:注意下面断言被注释掉了。
				//assert(test_sum != 3 && "Some error in compute local point feature histogram.");
				output.push_back(tmp_signature);
			}
			else
			{
				std::cout << "error:" << kdtree_ptr_->radiusSearch(input_ptr_->at(i), r_, pointIdx_vector, pointDistance_vector)
					<< std::endl;
			}
		}
#ifdef _DEBUG
		std::cout << "1:" << index_1_min << ":" << index_1_max << std::endl;
		std::cout << "2:" << index_2_min << ":" << index_2_max << std::endl;
		std::cout << "3:" << index_3_min << ":" << index_3_max << std::endl;
#endif
		return true;
	}

	template <typename PointInT, typename PointOutT>
	double LFSH<PointInT, PointOutT>::dot(Eigen::Vector3f x, Eigen::Vector3f y)
	{
		return x(0) * y(0) + x(1) * y(1) + x(2) * y(2);
	}

	template <typename PointInT, typename PointOutT>
	int LFSH<PointInT, PointOutT>::computeLocalDepth(
		Eigen::Vector3f source_normal,
		Eigen::Vector3f source_point,
		Eigen::Vector3f target_point) const
	{
		double d(r_);
		d -= (dot(source_normal, target_point - source_point));
		return int(d / 2 / r_ * N1_);
	}

	template <typename PointInT, typename PointOutT>
	int LFSH<PointInT, PointOutT>::computeDeviationAngle(
		Eigen::Vector3f source_normal,
		Eigen::Vector3f target_normal) const
	{
		double theta(0.0);
		double l_s(0.0);
		double l_t(0.0);
		l_s = sqrt(dot(source_normal, source_normal));
		l_t = sqrt(dot(target_normal, target_normal));
		if (dot(target_normal, source_normal) > 0.9)
		{
			theta = 0.0;
		}
		else
		{
			theta = acos(abs(dot(target_normal, source_normal) / l_s / l_t));
		}
		if (theta < 0 || theta > M_PI)
			theta = M_PI;
		//std::cout << "m_pi" << M_PI << std::endl;
		if (theta < 0.00001) return 0;//TODO:先对付着，要找一下异常值得原因
		return int(theta / M_PI * N2_);
	}

	template <typename PointInT, typename PointOutT>
	int LFSH<PointInT, PointOutT>::computeDensity(
		Eigen::Vector3f source_normal,
		Eigen::Vector3f source_point,
		Eigen::Vector3f target_point) const
	{
		double d(0.0);
		source_point = source_point - target_point;
		d = sqrt((source_point(0) * source_point(0) + source_point(1) * source_point(1) + source_point(2) * source_point(2))
			- pow(dot(source_point, source_normal), 2));
		return int(d / r_ * N3_);
	}

	template <typename PointInT, typename PointOutT>
	bool LFSH<PointInT, PointOutT>::DisplayInput()
	{
		view_shared_ptr_->addPointCloud(input_ptr_);
		return true;
	}
};

bool ModelTest_LFSH()
{
	//Load data
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	

	//Build transform matrix.
	Eigen::Matrix4f transform_matrix(Eigen::Matrix4f::Identity());

	double theta = M_PI / 4.0 / 1.0;

	transform_matrix(0, 0) = cos(theta);
	transform_matrix(0, 1) = -sin(theta);
	transform_matrix(1, 0) = sin(theta);
	transform_matrix(1, 1) = cos(theta);

	//pcl::transformPointCloud(*src_ptr, *target_ptr, transform_matrix);

	//DownSample the point cloud.
	pcl::ApproximateVoxelGrid<pcl::PointXYZ>::Ptr compress_ptr(new pcl::ApproximateVoxelGrid<pcl::PointXYZ>);
	for (int step(1); step < 20; ++step)
	{	
		//Reload point cloud.
		pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture5_B.pcd", *src_ptr);

		double leaf_size(0.1);
		leaf_size -= step * 0.004;
		compress_ptr->setLeafSize(leaf_size, leaf_size, leaf_size);
		compress_ptr->setDownsampleAllData(true);

		compress_ptr->setInputCloud(src_ptr);
		compress_ptr->filter(*src_ptr);

		//Transform matrix
		pcl::transformPointCloud(*src_ptr, *target_ptr, transform_matrix);

		//Compute feature
		pcl::LFSH<pcl::PointXYZ, pcl::LFSHSignature> lfsh_extract;

		pcl::PointCloud<pcl::LFSHSignature>::Ptr src_lfsh_ptr(new pcl::PointCloud<pcl::LFSHSignature>);
		pcl::PointCloud<pcl::LFSHSignature>::Ptr target_lfsh_ptr(new pcl::PointCloud<pcl::LFSHSignature>);

		lfsh_extract.setInputCloud(src_ptr);
		lfsh_extract.compute(*src_lfsh_ptr);

		lfsh_extract.setInputCloud(target_ptr);
		lfsh_extract.compute(*target_lfsh_ptr);
		double avg(0.0);
		for (int i(0); i < target_lfsh_ptr->size(); ++i)
		{
			double sum(0.0);
			for (int j(0); j < 33; ++j)
			{
				sum += pow(target_lfsh_ptr->at(i).histogram[j] - src_lfsh_ptr->at(i).histogram[j], 2);
			}
			avg += sum;

			//std::cout << "[" << i << "]:" << pow(sum, 0.5) << std::endl;
		}
		avg = avg / target_lfsh_ptr->size();
		std::cout << "leaf size:" << leaf_size << "avg:" << avg << std::endl;
	}


	int a(0);
	std::cin >> a;


	return true;
}


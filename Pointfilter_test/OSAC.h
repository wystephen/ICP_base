#pragma once

#include"LFSH.h"

#include <pcl/filters/approximate_voxel_grid.h>

#include "kdtree_feature.h"

#include "LM6DOF.h"

#include <pcl/registration/registration.h>
#include <pcl/registration/ia_ransac.h>

#include "RANSAC.h"



//For random generation
#include <stdlib.h>
#include <time.h>

#define IS_USE_KDTREE false
#define BIG_FLAT 99999999.0f


//As Kd-tree algorithm in pcl maybe can't use to search LFSHSignature type.
namespace pcl
{
	template <typename PointSourceT, typename FeatureType>
	class OSAC
	{
		/***************************
		FeatureType 可以使用有histogram 成员的pcl点类型，
		应该要有返回hist size 的函数 getNumberOfDimensions() 
		******************************/
	public:
		typedef boost::shared_ptr<pcl::PointCloud<PointSourceT>> PointCloudPtr;
		typedef const boost::shared_ptr<pcl::PointCloud<PointSourceT>> PointCloudConstPtr;

		typedef boost::shared_ptr<pcl::PointCloud<FeatureType>> LPFHPointCloudPtr;

		typedef boost::shared_ptr<pcl::search::KdTree<pcl::LFSHSignature>> KDTREEPtr;

		typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr;

		OSAC(): delta_(0.3), src_compress_ptr_(new pcl::PointCloud<PointSourceT>),
		        target_compress_ptr_(new pcl::PointCloud<PointSourceT>),
		        src_feature_ptr_(new pcl::PointCloud<FeatureType>),
		        target_feature_ptr_(new pcl::PointCloud<FeatureType>),
		        kd_ptr_(new pcl::kdtree_feature<pcl::LFSHSignature>)
		{
			srand(int(time(0)));//Set seed to get random number.

			//set std transform matrix
			Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

			double theta = M_PI / 4.0/1.0;

			transform_matrix(0, 0) = cos(theta);
			transform_matrix(0, 1) = -sin(theta);
			transform_matrix(1, 0) = sin(theta);
			transform_matrix(1, 1) = cos(theta);

			transform_matrix(0, 3) = 4;
			transform_matrix(1, 3) = 4;

			std_matrix_ = transform_matrix;

			compress_size_ = 0.01;

#ifdef _DEBUG
			compress_size_ = 0.05;
#endif

			N_ = 20;
			FeatureType tmp;
			feature_dimension_ = tmp.getNumberOfDimensions();

			alpha_ = 1.0;

			compress_grid_.setLeafSize(compress_size_, compress_size_, compress_size_);
			compress_grid_.setDownsampleAllData(true);

			d_tua_ = 1.3;//

			N_iter = 3000;

			x_ = 60;
			d_min_ = 0.05;	//invaild,change in feature extraction function.
		}


		~OSAC();
		std::vector<PointSourceT> src_vector;
		std::vector<PointSourceT> target_vector;

		/*************************************************
		  parameter set functions
		**************************************************/
		bool compute(Eigen::Matrix4f& transform_matrix);

		//This function is just to test my new idea.
		bool RANSACcompute(Eigen::Matrix4f& transform_matrix);

		bool setSourceCloud(const PointCloudPtr source);

		bool setTargetCloud(const PointCloudPtr target);

		bool ModelTest_OSAC();

		double getCompressSize() const;
		double setCompressSize(double size);

		int getNumberOfCorr() const;
		int setNumberOfCorr(int N);

		double setAlpha(double alpha);
		double getAlpha() const;


		//save correspondence find in correspondence generation
		struct mutlicorr
		{
			int src_index_;
			std::vector<int> target_index_;
			std::vector<float> target_dists_;
		};

	protected:

		double delta_;//in eq(8).(default: 0.3)
		double d_tua_;//in eq(8),As a threshold to choice a subset from all of P_1.

		int x_;//Select x_ samples from C.

		double avg_dis_; //avg distance get in feature histogram
		double std_dev_; // standardra division of  feature histogram

		double d_min_;//User-defined minimum distance for select point from pairwise.

		int N_iter;//Step conditions for iteratioin.

		std::vector<mutlicorr> correspon_vecotr_;

		double compress_size_; //The size of voxel use to compress pointclouds.(default:0.01)
		int N_;//The number of nearest feature point find in Correspondence generation.
		int feature_dimension_; //histogram arrary's size.Update in create the class.

		double alpha_; // \alpha in eq(7),to compute the threshold d_j.(default:1)In paper,suggest it between 0.5 to 1.0.

		float d_f_;//threshold value to choice a subset of all correspondence.

		pcl::ApproximateVoxelGrid<PointSourceT> compress_grid_;

		PointCloudPtr src_compress_ptr_;
		PointCloudPtr target_compress_ptr_;

		LPFHPointCloudPtr src_feature_ptr_;
		LPFHPointCloudPtr target_feature_ptr_;

		boost::shared_ptr<pcl::kdtree_feature<FeatureType>> kd_ptr_;

		Eigen::Matrix4f std_matrix_;

	private:


		/******************************************************
		 In paper , A fast and robust local descriptor for 3D point cloud registration
		 ,4.2. Correspondence generation
		*******************************************************/
		bool CorrespondeceGeneration();

		bool FeatureExtraction();//Use LFSH class to extraction feature.

		//point to surface distance,eq(9).
		double dis_p2s(int src_i, std::vector<int> target_vector);

		double D_avg(std::vector<int> in_index, std::vector<int> out_index);

		bool OSAC_main(Eigen::Matrix4f& transfomr_matrix);

		double D_avg(std::vector<Eigen::Vector3f> in, std::vector<Eigen::Vector3f> target);

		bool RandSelect(Eigen::MatrixXf& src, Eigen::MatrixXf& target, std::vector<Eigen::Vector3f>& target_vector);

		Eigen::Matrix4f TransformSolve(Eigen::MatrixXf src,
		                               Eigen::MatrixXf target);
	};


	template <typename PointSourceT, typename FeatureType>
	OSAC<PointSourceT, FeatureType>::~OSAC()
	{
	}


	template <typename PointSourceT, typename FeatureType>
	bool OSAC<PointSourceT, FeatureType>::compute(Eigen::Matrix4f& transform_matrix)
	{
		correspon_vecotr_.clear();

		//Extraction  feature
		FeatureExtraction();
		/********************************** /
		//Correspondence generation	 and us
		CorrespondeceGeneration();

		//OSAC 
		OSAC_main(transform_matrix);
		std::cout << "osac_main_transform_matrix:" << std::endl;
		std::cout << transform_matrix << std::endl;
		/*************************************/

		//use correspondence in pcl
		/*********************** /
		pcl::registration::CorrespondenceEstimation<pcl::LFSHSignature, pcl::LFSHSignature, float> 
			corr_estimation;
		corr_estimation.setInputCloud(src_feature_ptr_);
		corr_estimation.setInputTarget()
		corr_estimation.setSearchMethodTarget(kd_ptr_);
		pcl::Correspondence corr_data;
		corr_estimation.determineCorrespondences(corr_data,0.1);
		/****************************************** /
		//    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_fpfh_ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh_ptr(new pcl::PointCloud<pcl::FPFHSignature33>);
		src_fpfh_ptr->resize(src_feature_ptr_->size());
		target_fpfh_ptr->resize(target_feature_ptr_->size());
		for (int i(0); i < src_feature_ptr_->size();++i)
		{
			for (int j(0); j < 30;++j)
			{
				src_fpfh_ptr->at(i).histogram[j] = src_feature_ptr_->at(i).histogram[j];
			}
			src_fpfh_ptr->at(i).histogram[30] = 0.0;
			src_fpfh_ptr->at(i).histogram[31] = 0.0;
			src_fpfh_ptr->at(i).histogram[32] = 0.0;
		}
		for (int i(0); i < target_feature_ptr_->size();++i)
		{
			for (int j(0); j < 30;++j)
			{
				target_fpfh_ptr->at(i).histogram[j] = target_feature_ptr_->at(i).histogram[j];
			}
			target_fpfh_ptr->at(i).histogram[30] = 0.0;
			target_fpfh_ptr->at(i).histogram[31] = 0.0;
			target_fpfh_ptr->at(i).histogram[32] = 0.0;
		}
		/********************* /
		//pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::LFSHSignature> sac_ia;
		//重载 KdTreeFLANN····；《LFSHSignature；
		//sac_ia.setInputSource(src_compress_ptr_);
		//sac_ia.setSourceFeatures(src_fpfh_ptr);
		//sac_ia.setSourceFeatures(src_feature_ptr_);
		sac_ia.setInputTarget(target_compress_ptr_);
		//sac_ia.setTargetFeatures(target_fpfh_ptr);
		sac_ia.setTargetFeatures(target_feature_ptr_);

		pcl::PointCloud<pcl::PointXYZ> re;
		sac_ia.align(re);
		transform_matrix = sac_ia.getFinalTransformation();
		std::cout << transform_matrix << std::endl;
		/**********************/
		pcl::RANSAC<PointSourceT, FeatureType> ransac;
		ransac.setData(*src_compress_ptr_,
			*target_compress_ptr_,
			*src_feature_ptr_,
			*target_feature_ptr_,
			*kd_ptr_);	 
		//Input Date to ransac class which is similar to the ransac algorithm in PCL



		return true;
	}

	template <typename PointSourceT, typename FeatureType>
	bool OSAC<PointSourceT, FeatureType>::setSourceCloud(const PointCloudPtr source)
	{
		compress_grid_.setInputCloud(source);
		compress_grid_.filter(*src_compress_ptr_);

		if (src_compress_ptr_->size() > 0)
		{
			return true;
		}
		else
		{
			std::cout << "size after compress:" << src_compress_ptr_->size() << std::endl;
			return false;
		}
	}

	template <typename PointSourceT, typename FeatureType>
	bool OSAC<PointSourceT, FeatureType>::setTargetCloud(const PointCloudPtr target)
	{
		compress_grid_.setInputCloud(target);
		compress_grid_.filter(*target_compress_ptr_);
		if (target_compress_ptr_->size() > 0)
		{
			//kd_ptr_->setInputCloud(target_compress_ptr_);
			return true;
		}
		else
		{
			std::cout << "size after compress:" << target_compress_ptr_->size() << std::endl;
			return false;
		}
	}


	template <typename PointSourceT, typename FeatureType>
	double OSAC<PointSourceT, FeatureType>::getCompressSize() const
	{
		return compress_size_;
	}

	template <typename PointSourceT, typename FeatureType>
	double OSAC<PointSourceT, FeatureType>::setCompressSize(double size)
	{
		compress_size_ = size;
		compress_grid_.setLeafSize(size, size, size);
		return compress_size_;
	}

	template <typename PointSourceT, typename FeatureType>
	int OSAC<PointSourceT, FeatureType>::getNumberOfCorr() const
	{
		return N_;
	}

	template <typename PointSourceT, typename FeatureType>
	int OSAC<PointSourceT, FeatureType>::setNumberOfCorr(int N)
	{
		N_ = N;
		return N_;
	}

	template <typename PointSourceT, typename FeatureType>
	double OSAC<PointSourceT, FeatureType>::setAlpha(double alpha)
	{
		alpha_ = alpha;
		return alpha_;
	}

	template <typename PointSourceT, typename FeatureType>
	double OSAC<PointSourceT, FeatureType>::getAlpha() const
	{
		return alpha_;
	}

	template <typename PointSourceT, typename FeatureType>
	bool OSAC<PointSourceT, FeatureType>::CorrespondeceGeneration()
	{
		for (int i(0); i < src_compress_ptr_->size(); ++i)
		{
			mutlicorr tmp_mutlicorr;
			tmp_mutlicorr.src_index_ = i;
			kd_ptr_->searchNNearest(
				src_feature_ptr_->at(i), N_,
				tmp_mutlicorr.target_index_,
				tmp_mutlicorr.target_dists_);
			for (int j(0); j < N_; ++j)
			{
				avg_dis_ += tmp_mutlicorr.target_dists_[j];
			}
			correspon_vecotr_.push_back(tmp_mutlicorr);
		}
		avg_dis_ = avg_dis_ / (correspon_vecotr_.size() * N_);
		std::cout << "avg_dis:" << avg_dis_ << std::endl;

		//TODO:先不管效率，暴力计算出最合理整体的方差
		for (int i(0); i < correspon_vecotr_.size(); ++i)
		{
			for (int j(0); j < N_; ++j)
			{
				std_dev_ += pow(correspon_vecotr_[i].target_dists_[j] - avg_dis_, 2);
			}
		}
		std_dev_ = std_dev_ / (correspon_vecotr_.size() * N_);
		std::cout << "std_dev_:" << std_dev_ << std::endl;

		d_f_ = avg_dis_ - alpha_ * std_dev_;
		long sum_delete(0);
		//根据论文 eq（7） 排除 dis 大于 d_f_的点。
		for (int i(0); i < correspon_vecotr_.size(); ++i)
		{
			std::vector<int>::iterator it_index = correspon_vecotr_[i].target_index_.begin();
			std::vector<float>::iterator it_dist = correspon_vecotr_[i].target_dists_.begin();
			while (it_index != correspon_vecotr_[i].target_index_.end())
			{
				if (*it_dist > d_f_)
				{
					it_dist = correspon_vecotr_[i].target_dists_.erase(it_dist);
					it_index = correspon_vecotr_[i].target_index_.erase(it_index);

					++sum_delete;
				}
				else
				{
					++it_dist;
					++it_index;
				}
			}
		}
		//重构的时候注意这个地方需要与上边的循环合并，应该是利用迭代器直接迭代和erase。
		//处理删除了 target中所有元素的情况
		//std::cout << "Before correspon_vector_.size():" << correspon_vecotr_.size() << std::endl;
		std::vector<mutlicorr>::iterator it = correspon_vecotr_.begin();
		while (it != correspon_vecotr_.end())
		{
			if (it->target_index_.size() == 0)
			{
				it = correspon_vecotr_.erase(it);
			}
			else
			{
				++it;
			}
		}
		//std::cout << "After correspon_vector_.size():" << correspon_vecotr_.size() << std::endl;

		//d_tua_ = avg_dis_;//TODO: Remenber this update.

		std::cout << "sum_delete:" << sum_delete << std::endl;
		if (correspon_vecotr_.size() == src_compress_ptr_->size())
		{
			return true;
		}
		else
		{
			std::cout << "Error:correspon number != src number" << std::endl;
			return false;
		}
	}

	template <typename PointSourceT, typename FeatureType>
	double OSAC<PointSourceT, FeatureType>::dis_p2s(
		int src_i,
		std::vector<int> target_vector)
	{
		double distance(10000.0);
		double dis_tmp(1000);
		PointSourceT a, b;
		a = src_compress_ptr_->at(src_i);
		for (int i(0); i < target_vector.size(); ++i)
		{
			b = target_compress_ptr_->at(target_vector[i]);

			dis_tmp = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
			if (dis_tmp < distance)
			{
				distance = dis_tmp;
			}
		}
		return distance;
	}


	template <typename PointSourceT, typename FeatureType>
	double OSAC<PointSourceT, FeatureType>::D_avg(
		std::vector<int> in_index,
		std::vector<int> out_index)
	{
		int N_q(0);
		double sum_dis(0.0);
		double tmp_dis(0.0);
		for (int i(0); i < in_index.size(); ++i)
		{
			tmp_dis = dis_p2s(in_index[i], out_index);
			if (tmp_dis < d_tua_)
			{
				sum_dis += tmp_dis;
				N_q++;
			}
		}
		if (N_q / std::min(in_index.size(), out_index.size()) > delta_ && N_q > 0)
		{
			return sum_dis / double(N_q);
		}
		else
		{
			return BIG_FLAT;
		}
	}

	template <typename PointSourceT, typename FeatureType>
	bool OSAC<PointSourceT, FeatureType>::OSAC_main(Eigen::Matrix4f& transfomr_matrix)
	{
		Eigen::Matrix4f tmp_transform(Eigen::Matrix4f::Identity());

		double min_D_avg(0);
		double min_err(1000);

		double min_matrix_dis(100000000);
		Eigen::Matrix4f min_matrix;

		for (int i(0); i < N_iter; ++i)
		{
			Eigen::MatrixXf source(x_, 4);
			Eigen::MatrixXf target(x_, 4);

			std::vector<Eigen::Vector3f> in_tmp;
			std::vector<Eigen::Vector3f> out_tmp;

			//select correspondences
			RandSelect(source, target, out_tmp);

			//compute transform M_i
			tmp_transform = TransformSolve(source, target);

			//transform	
			source = tmp_transform * source.transpose(); //TODO:

			/********************************************/
			Eigen::MatrixXf err(x_, 4);
			err = source - target;
			double sum(0);
			for (int i(0); i < source.rows(); ++i)
			{
				sum += sqrt(pow(err(i, 0), 2) + pow(err(i, 1), 2) + pow(err(i, 2), 2));
			}
			//std::cout << "avg_dis_after_solve:" << sum / x_ << std::endl;


			/*******************************************************/

			//compute D_avg


			Eigen::Vector3f tmp;
			//std::cout << source.rows() << "x" << source.rows() << std::endl;
			assert(source.rows() == target.rows());
			for (int j(0); j < source.rows(); ++j)
			{
				tmp(0) = source(j, 0);
				tmp(1) = source(j, 1);
				tmp(2) = source(j, 2);
				in_tmp.push_back(tmp);
			}

			double the_D_avg = D_avg(in_tmp, out_tmp);
			//TODO：Use all point with transform to compute D_avg.

			if (i == 0)
			{
				src_vector.clear();
				target_vector.clear();
				for (int k(0); k < source.rows(); ++k)
				{
					src_vector.push_back(PointSourceT(source(k, 0), source(k, 1), source(k, 2)));
					target_vector.push_back(PointSourceT(target(k, 0), target(k, 1), target(k, 2)));
				}

				min_D_avg = the_D_avg;
				transfomr_matrix = tmp_transform;
			}
			else if ((the_D_avg < min_D_avg))//|| (sum / x_ < min_err))//&& sum/x_ > 0.001))
			{
				min_err = sum / x_; //


				src_vector.clear();
				target_vector.clear();
				for (int k(0); k < source.rows(); ++k)
				{
					src_vector.push_back(PointSourceT(source(k, 0), source(k, 1), source(k, 2)));
					target_vector.push_back(PointSourceT(target(k, 0), target(k, 1), target(k, 2)));
				}
				min_D_avg = the_D_avg;
				transfomr_matrix = tmp_transform;

				double t_dis(0.0);
				double t_dis_d1(0.0), t_dis_d2(0.0);
				for (int ti(0); ti < 4;++ti)
				{
					for (int tj(0); tj < 4;++tj)
					{
						t_dis += pow(tmp_transform(ti, tj)-std_matrix_(ti, tj),2);

					}
				}
				
				if (t_dis < min_matrix_dis)
				{
					min_matrix = tmp_transform;
					min_matrix_dis = t_dis;
				}

			}
		}
		std::cout << std_matrix_ << std::endl;
		std::cout << "min matrix" << std::endl;
		std::cout << min_matrix << std::endl;
		std::cout << " min errr:" << min_err << std::endl;
		std::cout << "min D avg:" << min_D_avg << std::endl;

		return true;
	}

	template <typename PointSourceT, typename FeatureType>
	double OSAC<PointSourceT, FeatureType>::D_avg(std::vector<Eigen::Vector3f> in,
	                                              std::vector<Eigen::Vector3f> target)
	{
		int N_q(0);
		double sum_dis(0.0);
		double tmp_dis(1111110.0);
		//std::cout << "in size:" << in.size() << std::endl;
		for (int i(0); i < in.size(); ++i)
		{
			for (int j(0); j < target.size(); ++j)
			{
				Eigen::Vector3f a = in[i];
				Eigen::Vector3f b = target[j];

				double dis = sqrt(a(0) - b(0)) * (a(0) - b(0)) +
					(a(1) - b(1)) * (a(1) - b(1)) +
					(a(2) - b(2)) * (a(2) - b(2));
				if (dis < tmp_dis)
				{
					tmp_dis = dis;
				}
			}
			if (tmp_dis < d_tua_)
			{
				sum_dis += tmp_dis;
				N_q++;
			}
		}
		int min_size = in.size();
		if (min_size > target.size()) min_size = target.size();
		if ((N_q / min_size) > delta_ && N_q > 0)
		{
			return sum_dis / double(N_q);
		}
		else
		{
			return BIG_FLAT;
		}
	}

	template <typename PointSourceT, typename FeatureType>
	inline bool OSAC<PointSourceT, FeatureType>::RandSelect(Eigen::MatrixXf& src,
	                                                        Eigen::MatrixXf& target,
	                                                        std::vector<Eigen::Vector3f>& target_vector)
	{
		//先实现一个粗略的，有空重构一下代码。
		int add_times(0);
		while (add_times < x_)
		{ //第一步，随机选一个点，加入矩阵
			int the_index = Random(correspon_vecotr_.size());
			if (add_times == 0)
			{
				src(add_times, 0) = src_compress_ptr_->at(correspon_vecotr_[the_index].src_index_).x;
				src(add_times, 1) = src_compress_ptr_->at(correspon_vecotr_[the_index].src_index_).y;
				src(add_times, 2) = src_compress_ptr_->at(correspon_vecotr_[the_index].src_index_).z;
				src(add_times, 3) = 1.0;

				PointSourceT p;
				p = target_compress_ptr_->at(correspon_vecotr_[the_index].target_index_[
					Random(correspon_vecotr_[the_index].target_index_.size())]);
				target(add_times, 0) = p.x;
				target(add_times, 1) = p.y;
				target(add_times, 2) = p.z;
				target(add_times, 3) = 1.0;

				add_times++;
				//TODO:异常处理，是不是考虑 target vector 为空
			}
			else
			{
				//第二部，选择点需要与已经选择的点距离大于一定值.此处逻辑显然可优化

				bool IsOk(true);
				PointSourceT s_p;
				s_p = src_compress_ptr_->at(correspon_vecotr_[the_index].src_index_);
				for (int k(0); k < add_times; ++k)
				{
					if (
						(pow(s_p.x - src(k, 0), 2) + pow(s_p.y - src(k, 1), 2) + pow(s_p.z - src(k, 2), 2))
						< (d_min_ * d_min_)
					)
					{
						IsOk = false;
					}
				}
				if (IsOk)
				{
					PointSourceT t_p;
					bool IsOk_target(true);
					for (int idx(0); idx < correspon_vecotr_[the_index].target_index_.size(); ++idx)
					{
						t_p = target_compress_ptr_->at(correspon_vecotr_[the_index].target_index_[idx]);

						for (int k(0); k < add_times; ++k)
						{
							if (
								pow(t_p.x - target(k, 0), 2) +
								pow(t_p.y - target(k, 1), 2) +
								pow(t_p.z - target(k, 2), 2)
								< (d_min_ * d_min_)
							)
							{
								IsOk_target = false;
							}
						}
						if (IsOk_target)
						{
							src(add_times, 0) = s_p.x;
							src(add_times, 1) = s_p.y;
							src(add_times, 2) = s_p.z;
							src(add_times, 3) = 1.0;

							target(add_times, 0) = t_p.x;
							target(add_times, 1) = t_p.y;
							target(add_times, 2) = t_p.z;
							target(add_times, 3) = 1.0;
							for (int idx(0); idx < correspon_vecotr_[the_index].target_index_.size(); ++idx)
							{
								t_p = target_compress_ptr_->at(correspon_vecotr_[the_index].target_index_[idx]);
								Eigen::Vector3f vc(t_p.x, t_p.y, t_p.z);
								target_vector.push_back(vc);
							}

							++add_times;
							break;
						}
					}
				}
			}
		}
		return true;
	}

	template <typename PointSourceT, typename FeatureType>
	Eigen::Matrix4f OSAC<PointSourceT, FeatureType>::TransformSolve(Eigen::MatrixXf src, Eigen::MatrixXf target)
	{
		Eigen::Matrix4f transform;

		LM6DOF function(src.rows(), src, target);
		Eigen::LevenbergMarquardt<LM6DOF, float> lm(function);

		Eigen::VectorXf x;
		x.resize(6);
		for (int j(0); j < 6; ++j)
		{
			x(j) = 0;
		}

		lm.minimize(x);

		transform = function.computeTransformMatrix(x);

		return transform;
	}


	template <typename PointSourceT, typename FeatureType>
	bool OSAC<PointSourceT, FeatureType>::FeatureExtraction()
	{
		pcl::LFSH<pcl::PointXYZ, pcl::LFSHSignature> lfsh_extraction;
		lfsh_extraction.setInputCloud(src_compress_ptr_);
		lfsh_extraction.compute(*src_feature_ptr_);

		lfsh_extraction.setInputCloud(target_compress_ptr_);
		lfsh_extraction.compute(*target_feature_ptr_);

		d_min_ = lfsh_extraction.getModelSize() * 2/ x_;	 //TODO:self adjust d_min_ use Model_size
		std::cout << "d_min_:" << d_min_ << std::endl;
		kd_ptr_->setInputCloud(target_feature_ptr_);

		return true;
	}

	template <typename PointSourceT, typename FeatureType>
	bool OSAC<PointSourceT, FeatureType>::ModelTest_OSAC()
	{
		//Load Data
		pcl::PointCloud<pcl::PointXYZ>::Ptr p_src_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ>("write_capture5_B.pcd", *p_src_ptr);

		//Transform matrix
		Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

		double theta = M_PI / 4.0 / 1.0;

		transform_matrix(0, 0) = cos(theta);
		transform_matrix(0, 1) = -sin(theta);
		transform_matrix(1, 0) = sin(theta);
		transform_matrix(1, 1) = cos(theta);

		//Compress
		double leaf_size(0.025);
		compress_grid_.setLeafSize(leaf_size,leaf_size,leaf_size);

		compress_grid_.setInputCloud(p_src_ptr);
		compress_grid_.filter(*src_compress_ptr_);
		//Transform
		pcl::transformPointCloud(*src_compress_ptr_, *target_compress_ptr_, transform_matrix);

		std::cout << "Is src size = target size:" << 
			(src_compress_ptr_->size() == target_compress_ptr_->size()) << std::endl;

////////function in compute();
		correspon_vecotr_.clear();

		//Extraction  feature
		FeatureExtraction();


		//Correspondence generation	 and us
		CorrespondeceGeneration();

		//Test correspondence
		double threold_dis(0.021);
		threold_dis = threold_dis * threold_dis;
		int good_corre(0);
		for (int i(0); i < correspon_vecotr_.size();++i)
		{
			bool Isgood(false);
			pcl::PointXYZ p_s;
			p_s = target_compress_ptr_->at(correspon_vecotr_[i].src_index_);
			for (int j(0); j < correspon_vecotr_[i].target_index_.size();++j)
			{
				pcl::PointXYZ p_t(target_compress_ptr_->at(correspon_vecotr_[i].target_index_[j]));
				if ((pow(p_t.x-p_s.x,2)+pow(p_t.y-p_s.y,2)+pow(p_t.z-p_s.z,2))<threold_dis)
				{
					Isgood = true;
				}
			}
			if (Isgood) good_corre++;
		}
		std::cout << "%:" << good_corre <<":"<< correspon_vecotr_.size() << std::endl;

		//OSAC 
		//OSAC_main(transform_matrix);
		std::cout << "osac_main_transform_matrix:" << std::endl;
		std::cout << transform_matrix << std::endl;
		
		std::string a;
		std::cin >> a;
		return true;
	}
}





#include "stdafx.h"
#include "ICP.h"


template <typename PointSource, typename PointTarget>
ICP<PointSource, PointTarget>::~ICP()
{
}

template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::setSource(const PointCloudConstPtr& in)
{
	//std::cout << in->size() << std::endl;
	//std::cout << "in0at:" << in->at(2) << std::endl;
	//source_ptr_ = in;
	for (int k(0); k < in->size(); k++)
	{
		//std::cout << k << ":" << in->at(k) << std::endl;
		source_ptr_->push_back(in->at(k));
	}
	//std::cout << "set Source end!" << std::endl;
}

template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::setTarget(const PointCloudConstPtr& target)
{
	//std::cout << "target" << std::endl;
	//std::cout << target->size() << std::endl;
	for (int k(0); k < target->size(); k++)
	{
		//std::cout << k << ":" << target->at(k) << std::endl;
		target_ptr_->push_back(target->at(k));
	}
	//std::cout << "setTarget end:" << std::endl;
	kdtree_target_ptr_->setInputCloud(target_ptr_);
	//std::cout << "kd_tree build" << std::endl;

}

template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::computerTransformation(pcl::PointCloud<PointSource>& output,
                                                           const Eigen::Matrix4f& guess)
{
	pcl::PointCloud<PointSource>::Ptr input_transformed_ptr(new pcl::PointCloud<PointSource>);

	nr_iterations_ = 0;
	converged_ = false;

	final_transfrom_matrix = guess;

	if (guess != Eigen::Matrix4f::Identity())
	{
		input_transformed_ptr->resize(source_ptr_->size());
		transfromPointCloud(source_ptr_, *input_transformed_ptr, guess);
	}
	else
	{
		*input_transformed_ptr = *source_ptr_;
	}

	  transform_matrix = Eigen::Matrix4f::Identity();
	do
	{	
		
		//find correspondence
		simpleFindCorrespondence(input_transformed_ptr);
		//UsePCLFindCorrespondence(input_transformed_ptr);
		//solve transfrom LM;
		//getTransLM();
		getTransQR();

		transfromPointCloud(input_transformed_ptr, *input_transformed_ptr, transform_matrix);
	}
	while (!converged_);
}

template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::transfromPointCloud(typename pcl::PointCloud<PointSource>::Ptr input, pcl::PointCloud<PointSource>& output, const Eigen::Matrix4f& transform_matrix)
{
	//std::cout << "transfromPointCLoud" << std::endl;
	Eigen::Vector4f pt(0.0, 0.0, 0.0, 1.0), pt_t;
	Eigen::Matrix4f tr = transform_matrix.template cast<float>();


	size_t x_index_offset_, y_index_offset_, z_index_offset_;
	std::vector<pcl::PCLPointField> fields;
	pcl::getFields(*input, fields);
	for (int i(0); i < fields.size(); i++)
	{
		if (fields[i].name == "x") x_index_offset_ = fields[i].offset;
		else if (fields[i].name == "y") y_index_offset_ = fields[i].offset;
		else if (fields[i].name == "z") z_index_offset_ = fields[i].offset;
	}

	//Just use x y z 
	for (size_t i = 0; i < input->size(); ++i)
	{
		const uint8_t* data_in = reinterpret_cast<const uint8_t*>(&(input->at(i)));
		uint8_t* data_out = reinterpret_cast<uint8_t*>(&output[i]);

		memcpy(&pt[0], data_in + x_index_offset_, sizeof(float));
		memcpy(&pt[1], data_in + y_index_offset_, sizeof(float));
		memcpy(&pt[2], data_in + z_index_offset_, sizeof(float));

		if (!pcl_isfinite(pt[0]) || !pcl_isfinite(pt[1]) || !pcl_isfinite(pt[2]))
		{
			continue;
		}

		pt_t = tr * pt;
		memcpy(data_out + x_index_offset_, &pt_t[0], sizeof(float));
		memcpy(data_out + y_index_offset_, &pt_t[1], sizeof(float));
		memcpy(data_out + z_index_offset_, &pt_t[2], sizeof(float));
	}
}

template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::simpleFindCorrespondence(typename pcl::PointCloud<PointSource>::Ptr input)
{
	std::cout << "Find Correspondence" << std::endl;
	corr_vector_.clear();

	pcl::PointXYZ last_search_point_xyz, search_point_xyz;

	std::vector<int> point_idx_vector;
	std::vector<float> point_dis_vector;

	last_search_point_xyz = pcl::PointXYZ(0.0, 0.0, 0.0);
	for (int index(0); index < input->size(); index++)
	{
		search_point_xyz = input->at(index);
		if ((pow((last_search_point_xyz.x - search_point_xyz.x), 2)
			+ pow((last_search_point_xyz.y - search_point_xyz.y), 2)
			+ pow((last_search_point_xyz.z - search_point_xyz.z), 2)) < threshold_sperated_distance_squre)
		{
			continue;
		}


		if (kdtree_target_ptr_->nearestKSearch(search_point_xyz, 1, point_idx_vector, point_dis_vector) > 0)
		{
			{
				last_search_point_xyz = input->at(point_idx_vector[0]);

				if ((pow(last_search_point_xyz.x - search_point_xyz.x, 2) +
					pow(last_search_point_xyz.y - search_point_xyz.y, 2) +
					pow(last_search_point_xyz.z - search_point_xyz.z, 2))>threshold_distance_squre)
					continue;

				corr_vector_.push_back(
					correspondence(input->at(point_idx_vector[0]).x,
					               input->at(point_idx_vector[0]).y,
					               input->at(point_idx_vector[0]).z,
					               last_search_point_xyz.x,
					               last_search_point_xyz.y,
					               last_search_point_xyz.z,
					               point_dis_vector[0])
				);
			}
		}
	}
	std::cout << "correspondence size:" << corr_vector_.size() << std::endl;
}

template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::FindCorrespondenceNormal(typename pcl::PointCloud<PointSource>::Ptr input)
{
	//select

	//match

	//weighting

	//Rejecting

}


template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::getTransLM()
{
	TransfromEstimationLM function(corr_vector_, corr_vector_.size());

	Eigen::LevenbergMarquardt<TransfromEstimationLM, float> lm(function);

	Eigen::VectorXf x(16);

	//Eigen::Matrix<float,16,1> x;
	for (int i(0); i < 4; i++)
	{
		for (int j(0); j < 4; j++)
		{
			x[i + j * 4] = transform_matrix(i, j);
		}
	}

	Eigen::VectorXf fvec(1);
	function(x, fvec);
	//std::cout << "before:" << fvec << std::endl;


	lm.minimize(x);

	function(x, fvec);
	//std::cout << "after:" << fvec << std::endl;
	for (int i(0); i < 4; i++)
	{
		for (int j(0); j < 4; j++)
		{
			transform_matrix(i, j) = x[i + j * 4];
		}
	}

	final_transfrom_matrix = transform_matrix * final_transfrom_matrix;
	//std::cout <<"error:"<< x << std::endl;
	std::cout << "transform_matrix:"<<std::endl << final_transfrom_matrix << std::endl;
}


template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::getTransQR()
{
	std::cout << "get TransQR" << std::endl;
	std::cout << "corre size:" << corr_vector_.size() << std::endl;
	Eigen::MatrixXf A(corr_vector_.size(), 4);
	Eigen::MatrixXf b(corr_vector_.size(), 4);

	for (int i(0); i < corr_vector_.size(); i++)
	{
		A.row(i) << corr_vector_[i].x1 , corr_vector_[i].y1 , corr_vector_[i].z1 , 1.0;
		b.row(i) << corr_vector_[i].x2 , corr_vector_[i].y2 , corr_vector_[i].z2 , 1.0;
	}
	//Eigen::MatrixXf x = A.colPivHouseholderQr().solve(b);
	//Eigen::MatrixXf x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	Eigen::MatrixXf x = A.colPivHouseholderQr().solve(b);
	std::cout << x << std::endl;
	transform_matrix = x;
	final_transfrom_matrix = x * final_transfrom_matrix;
	std::cout << "final:" << final_transfrom_matrix << std::endl;
}

template <typename PointSource, typename PointTarget>
void ICP<PointSource, PointTarget>::getTransQR_cu()
{
	//try to use cuda QR to solve the transform from source point to target point.
}


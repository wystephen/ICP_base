#pragma once
#include <pcl/registration/icp.h>

class TransformLM
{
public:
	TransformLM() : src_matrix_ptr_(new Eigen::MatrixXf(200, 4)), target_matrix_ptr_(new Eigen::MatrixXf(200, 4))
	{
	}

	~TransformLM();
	bool setInputMatrix(Eigen::MatrixXf& input);
	bool setTargetMatrix(Eigen::MatrixXf& target) const;
	Eigen::Matrix4f compute();
private:
	boost::shared_ptr<Eigen::MatrixXf> src_matrix_ptr_;
	boost::shared_ptr<Eigen::MatrixXf> target_matrix_ptr_;

	Eigen::Matrix4f global_transform_matrix_;
	int N_itera;  //
	double x_offset_;
	double y_offset_;
	double z_offset_;
	double a_offset_;
	double b_offset_;
	double c_offset_;

	double funtion(double x, double y, double z, double a, double b, double c);
	Eigen::VectorXf diff_function(double x, double y, double z, double a, double b, double c);
	Eigen::Matrix4f makeMatrix(double x, double y, double z, double a, double b, double c);
};

inline bool TransformLM::setInputMatrix(Eigen::MatrixXf& input)
{
	*src_matrix_ptr_ = input;
	if (src_matrix_ptr_->rows() == input.rows())
		return true;
	else
		return false;
}

inline bool TransformLM::setTargetMatrix(Eigen::MatrixXf& target) const
{
	*target_matrix_ptr_ = target;
	if (target_matrix_ptr_->rows() == target.rows())
	{
		return true;
	}
	else
	{
		return false;
	}
}

Eigen::Matrix4f TransformLM::compute()
{
	
}

inline double TransformLM::funtion(double x, double y, double z, double a, double b, double c)
{
	Eigen::Matrix4f T;
	T = makeMatrix(x, y, z, a, b, c);
	//这里的norm是个函数，需要加括号。
	return (((*src_matrix_ptr_) * T) - (*target_matrix_ptr_)).norm() / (target_matrix_ptr_->norm());

}

inline Eigen::VectorXf TransformLM::diff_function(double x, double y, double z, double a, double b, double c)
{
	Eigen::VectorXf jacc;
	jacc.resize(6);
	double eps = 0.01;
	for (int i(0); i < 6;++i)
	{
		
	}

}

inline Eigen::Matrix4f TransformLM::makeMatrix(double x, double y, double z, double a, double b, double c)
{
	Eigen::Matrix4f T;
	T = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tmp_T(Eigen::Matrix4f::Identity());

	T(0, 3) = x;
	T(1, 3) = y;
	T(2, 3) = z;

	tmp_T(0, 0) = cos(a);
	tmp_T(0, 1) = sin(a);
	tmp_T(1, 0) = -sin(a);
	tmp_T(1, 1) = cos(a);

	T = T * tmp_T;
	tmp_T = Eigen::Matrix4f::Identity();

	tmp_T(1, 1) = cos(b);
	tmp_T(1, 2) = sin(b);
	tmp_T(2, 1) = -sin(b);
	tmp_T(2, 2) = cos(b);

	T = T*tmp_T;
	tmp_T = Eigen::Matrix4f::Identity();

	tmp_T(0, 0) = cos(c);
	tmp_T(0, 2) = sin(c);
	tmp_T(2, 0) = -sin(c);
	tmp_T(2, 2) = cos(c);
	T = T*tmp_T;
	return T;
}

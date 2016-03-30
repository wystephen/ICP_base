#pragma once
#include "C:\Users\WangYan\Documents\Visual Studio 2013\Projects\ICP_base\ICP_base\FUNCTOR.h"

#include <stdlib.h>
#include <time.h>

#define Random(range) (rand() % range)

#define  PI 3.1415926

class LM6DOF :
	public FUNCTOR<float>
{
public:
	typedef Eigen::VectorXf Vector6f;
	typedef boost::shared_ptr<Eigen::MatrixXf> MatrixPtr;

	LM6DOF(int data_size, Eigen::MatrixXf in, Eigen::MatrixXf target) : FUNCTOR<float>(6, data_size),
	                                                                    src_ptr_(new Eigen::MatrixXf),
	                                                                    target_ptr_(new Eigen::MatrixXf)
	{
		src_.resize(in.rows(), in.cols());
		target_.resize(target.rows(), target.cols());
		*src_ptr_ = in;
		*target_ptr_ = target;
		epsfcn = 0.000001;
	}

	~LM6DOF();
	Eigen::MatrixXf src_;
	Eigen::MatrixXf target_;
	MatrixPtr src_ptr_;
	MatrixPtr target_ptr_;
	float epsfcn;

	int operator()(const Eigen::VectorXf& x, Eigen::VectorXf& fvec);

	int df(const Eigen::VectorXf& x, Eigen::MatrixXf& jac);


	Vector6f valueCondition(Vector6f tf);

	Eigen::Matrix4f computeTransformMatrix(Eigen::VectorXf& tf);
};

inline LM6DOF::~LM6DOF()
{
}

inline int LM6DOF::operator()(const Eigen::VectorXf& x, Eigen::VectorXf& fvec)
{
	Eigen::Matrix4f tr;
	Vector6f t_x = x;
	
	tr = computeTransformMatrix(t_x);
	Eigen::MatrixXf err(src_.rows(), src_.cols());
	err = tr * (src_ptr_->transpose() ) - (*target_ptr_);  //Matrix multi.....
	for (int i(0); i < err.rows(); ++i)
	{
		fvec[i] = pow(err(i, 0), 2) + pow(err(i, 1), 2) + pow(err(i, 2), 2);
	}
	return 0;
}

inline int LM6DOF::df(const Eigen::VectorXf& x, Eigen::MatrixXf& jac)
{
	float h;
	int nfev = 0;

	const Eigen::VectorXf::Index n = 6;
	const float eps = sqrt((std::max)(epsfcn, Eigen::NumTraits<float>::epsilon()));

	Eigen::VectorXf val1, val2;
	Eigen::VectorXf tx = x;
	val1.resize(FUNCTOR::values());
	val2.resize(FUNCTOR::values());

	for (int j(0); j < n; ++j)
	{
		h = eps * abs(tx[j]);
		if (abs(h) < 0.000001)
		{
			h = eps;
		}
		h = abs(h);

		tx[j] += h;
		operator()(tx, val2);
		nfev++;
		tx[j] -= (2 * h);
		operator()(tx, val1);
		nfev++;
		tx[j] = x[j];
		jac.col(j) = (val2 - val1) / (2 * h);
		//std::cout << jac << std::endl;
	}
	return 0;
}

inline LM6DOF::Vector6f LM6DOF::valueCondition(Vector6f tf)
{
	for (int i(0); i < 3; ++i)
	{
		if (tf(i) < 0.0)
		{
			//tf(i) += int(abs(tf(i) / PI)) * PI;
		}
	}
	return tf;
}

Eigen::Matrix4f LM6DOF::computeTransformMatrix(Eigen::VectorXf& tf)
{
	tf = valueCondition(tf);
	Eigen::Matrix4f T;
	double coeef_(1);
	T = Eigen::Matrix4f::Identity();
	/****************************************************************** /
	Eigen::Matrix4f tmp_T(Eigen::Matrix4f::Identity());
	double coeef(1.0); 

	

	tmp_T(0, 0) = cos(tf(3));
	tmp_T(0, 1) = sin(tf(3));
	tmp_T(1, 0) = -sin(tf(3));
	tmp_T(1, 1) = cos(tf(3));

	tmp_T(0, 3) = 1;
	tmp_T(1, 3) = 1.0;
	tmp_T(2, 3) = 1.0;

	T = T	*   tmp_T;
	tmp_T = Eigen::Matrix4f::Identity();

	tmp_T(0, 0) = cos(tf(5));
	tmp_T(0, 2) = sin(tf(5));
	tmp_T(2, 0) = cos(tf(5));
	tmp_T(2, 2) = cos(tf(5));

	tmp_T(0, 3) = 1;
	tmp_T(1, 3) = 1.0;
	tmp_T(2, 3) = 1.0;

	T = T	*   tmp_T;
	tmp_T = Eigen::Matrix4f::Identity();

	tmp_T(1, 1) = cos(tf(4));
	tmp_T(1, 2) = sin(tf(4));
	tmp_T(2, 1) = -sin(tf(4));
	tmp_T(2, 2) = cos(tf(4));

	tmp_T(0, 3) = 1;
	tmp_T(1, 3) = 1.0;
	tmp_T(2, 3) = 1.0;
	T =  T	*   tmp_T ;
	T(0, 3) = tf(0)*coeef;
	T(1, 3) = tf(1)*coeef;
	T(2, 3) = tf(2)*coeef;
	T.normalized();
	/****************************************************************/
	double x, y, z, X, Y, Z;
	x = tf(0);
	y = tf(1);
	z = tf(2);
	X = tf(3)*coeef_;
	Y = tf(4)*coeef_;
	Z = tf(5)*coeef_;
	
	/*******************************
	x y z X Y Z
	cosx*cosz           sinx*siny*cosz-cosx*sinz        cosx*siny*cosz+sinx*sinz          X
	cony*sinz           sinx*siny*sinz+cosx*cosz       cosx*siny*sinz-sinx*cosz           Y
	-siny                   sinx*cosy                                 cosx*cosy           Z
	0                         0                        0                                  1
	*******************************/

	//row 1
	T(0, 0) = cos(x) * cos(z);
	T(0, 1) = sin(x) * sin(y) * cos(z) - cos(x)* sin(z);
	T(0, 2) = cos(x) * sin(y) * cos(z) + sin(x) * sin(z);
	T(0, 3) = X;

	//row 2
	T(1, 0) = cos(y) * sin(z);
	T(1, 1) = sin(x) * sin(y) * sin(z) + cos(x) * cos(z);
	T(1, 2) = cos(x) * sin(y) * sin(z) - sin(x) * cos(z);
	T(1, 3) = Y;

	//row 3
	T(2, 0) = -sin(y);
	T(2, 1) = sin(x) * cos(y);
	T(2, 2) = cos(x)* cos(y);
	T(2, 3) = Z;
	
	//row 4
	T(3, 0) = 0;
	T(3, 1) = 0;
	T(3, 2) = 0;
	T(3, 3) = 1;

	return T;
}

bool ModelTest_LM6DOF()
{

	//Build a transform matrix
	Eigen::Matrix4f transform_matrix(Eigen::Matrix4f::Identity());

	double theta = M_PI / 4.0 / 1.0;

	transform_matrix(0, 0) = cos(theta);
	transform_matrix(0, 1) = -sin(theta);
	transform_matrix(1, 0) = sin(theta);
	transform_matrix(1, 1) = cos(theta);

	transform_matrix(0, 3) = 1.0;
	transform_matrix(1, 3) = 1.0;
	transform_matrix(2, 3) = 0.7;

	//New Point src and target.

	Eigen::MatrixXf src_t, target_t;
	Eigen::Vector4f src_tmp;
	
	int data_size(20);
	src_t.resize(data_size, 4);
	target_t.resize(data_size, 4);

	srand(int(time(0)));//Set seed to get random number.

	for (int i(0); i < data_size;++i)
	{
		src_tmp(0) = double(Random(1000)) / 500.0;
		src_tmp(1) = double(Random(1000)) / 500.0;
		src_tmp(2) = double(Random(1000)) / 500.0;
		src_tmp(3) = 1.0;

		src_t.row(i) = src_tmp;

	}
	target_t = transform_matrix * src_t.transpose();//src_t * transform_matrix;
	for (int j(0); j < data_size;++j)
	{
		for (int i(0); i < 4;++i)
		{
			//target_t(j, i) += (double(Random(2000)) / 4000 - 0.25);
		}
	}
	//Compute transform
	Eigen::Matrix4f solveMatrix(Eigen::Matrix4f::Identity());
	LM6DOF func(src_t.rows(), src_t,target_t);
	Eigen::LevenbergMarquardt<LM6DOF, float> lm(func);


	Eigen::VectorXf x;
	x.resize(6);
	for (int k(0); k < 6;++k)
	{
		x(k) = 0.0;
	}

	lm.minimize(x);
	solveMatrix = func.computeTransformMatrix(x);

	std::cout << "transfomr matrix:" << std::endl;
	std::cout << transform_matrix << std::endl;

	std::cout << "solve matrix:" << std::endl;
	std::cout << solveMatrix << std::endl;



	//templete
	int a(0);
	std::cin >> a;
	return true;
}


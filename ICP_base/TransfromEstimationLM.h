#pragma once
#include "FUNCTOR.h"
#include "FUNCTOR.cpp"
#include "correspondence.h"

#include <boost\shared_ptr.hpp>
#include "math.h"
#include <vector>

#include <iostream>

struct TransfromEstimationLM :
	  FUNCTOR<float>
{
public:
	TransfromEstimationLM(std::vector<correspondence> corr_vec,int data_size) :FUNCTOR<float>(16, data_size) {
		correspondences_vector_ = corr_vec;
		//epsfcn = 0.01;
	}

	~TransfromEstimationLM(){};
	//typedef Eigen::VectorXf InputType;
	//typedef Eigen::VectorXf ValueTyep;

	//typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	std::vector<correspondence> correspondences_vector_;

	float epsfcn;

	int operator() (const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const 
	{
		Eigen::Matrix4f tr;
		for (int i(0); i < 4; i++)
		{
			for (int j(0); j < 4; j++)
			{
				tr(i, j) = x(i + j * 4);
			}


		}
		Eigen::Vector4f pt(0.0, 0.0, 0.0, 1.0), pt_t(0.0, 0.0, 0.0, 1.0);
		Eigen::Vector4f target(0.0, 0.0, 0.0, 1.0);

		for (int index(0); index < m_values; index++)
		{
			pt = Eigen::Vector4f(correspondences_vector_[index].x1, correspondences_vector_[index].y1,
				correspondences_vector_[index].z1, 1.0);
			pt_t = tr * pt;
			target = Eigen::Vector4f(correspondences_vector_[index].x2,
				correspondences_vector_[index].y2, correspondences_vector_[index].z2, 1.0);

			fvec[index] = pow(pt_t[0] - target[0], 2) + pow(pt_t[1] - target[1], 2) + pow(pt_t[2] - target[2], 2);
		}
		return 0;
	}

	int df(const Eigen::VectorXf &x, Eigen::MatrixXf &jac) const
	{
		//TransfromEstimationLM tm()
		//boost::shared_ptr<TransfromEstimationLM> tmp(this);
		//tmp = this;
		//Eigen::NumericalDiff<TransfromEstimationLM> numDiff(*this);
		//numDiff.df(x, jac);
		float h;
		int nfev = 0;
		const Eigen::VectorXf::Index n = 16;
		const float eps = sqrt((std::max)(epsfcn, Eigen::NumTraits<float>::epsilon()));
		std::cout << "eps:" << eps << std::endl;

		Eigen::VectorXf val1, val2;
		Eigen::VectorXf tx = x;

		val1.resize(FUNCTOR::values());
		val2.resize(FUNCTOR::values());


		operator ()(tx, val1);
		nfev++;

		for (int j(0); j < n; j++)
		{
			h = eps * abs(tx[j]);
			if (abs(h - 0.0)<0.00000001)
			{
				h = eps;
			}		
			tx[j] += h;
			operator ()(tx, val2);
			nfev++;
			tx[j] = x[j];
			jac.col(j) = (val2 - val1) / h;

		}
		//std::cout << "jac"<<jac << std::endl;

		
		return 0;
	};


};




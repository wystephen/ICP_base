#pragma once
#include "FUNCTOR.h"
#include "FUNCTOR.cpp"
#include "correspondence.h"

#include <boost\shared_ptr.hpp>
#include "math.h"
#include <vector>

struct TransfromEstimationLM :
	 public FUNCTOR<float>
{
public:
	TransfromEstimationLM(std::vector<correspondence> corr_vec,int data_size) :FUNCTOR<float>(16, data_size) {
		correspondences_vector_ = corr_vec;
	}

	~TransfromEstimationLM(){};


	std::vector<correspondence> correspondences_vector_;

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

	int df(const Eigen::VectorXf &x, JacobianType &jac) const
	{
		//TransfromEstimationLM tm()
		//boost::shared_ptr<TransfromEstimationLM> tmp(this);
		//tmp = this;
		Eigen::NumericalDiff<TransfromEstimationLM> numDiff(*this);
		numDiff.df(x, jac);
		return 0;
	};


};




#include "stdafx.h"
#include "TransfromEstimationLM.h"


//int TransfromEstimationLM::operator() (const InputType& x, ValueType& fvec)	const
//{
//	Eigen::Matrix4f tr;
//	for (int i(0); i < 4; i++)
//	{
//		for (int j(0); j < 4; j++)
//		{
//			tr(i, j) = x(i + j * 4);
//		}
//
//
//	}
//	Eigen::Vector4f pt(0.0, 0.0, 0.0, 1.0), pt_t(0.0, 0.0, 0.0, 1.0);
//	Eigen::Vector4f target(0.0, 0.0, 0.0, 1.0);
//
//	for (int index(0); index < m_values; index++)
//	{
//		pt = Eigen::Vector4f(correspondences_vector_[index].x1, correspondences_vector_[index].y1,
//			correspondences_vector_[index].z1, 1.0);
//		pt_t = tr * pt;
//		target = Eigen::Vector4f(correspondences_vector_[index].x2,
//			correspondences_vector_[index].y2, correspondences_vector_[index].z2, 1.0);
//
//		fvec[index] = pow(pt_t[0] - target[0], 2) + pow(pt_t[1] - target[1], 2) + pow(pt_t[2] - target[2], 2);
//	}
//	return 0;
//}
//
//int TransfromEstimationLM::df(const Eigen::VectorXf& x, JacobianType& jac) const
//{
//	//TransfromEstimationLM tm()
//	//boost::shared_ptr<TransfromEstimationLM> tmp(this);
//	//tmp = this;
//	Eigen::NumericalDiff<TransfromEstimationLM> numDiff(*this);
//	numDiff.df(x, jac);
//	return 0;
//}


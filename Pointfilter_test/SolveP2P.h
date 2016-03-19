#pragma once
#include "GridTransformationSolveBase.h"
class SolveP2P :
	public slam::GridTransformationSolveBase
{
public:
	
	typedef boost::shared_ptr<Eigen::MatrixXf> PointDataPtr;

	SolveP2P() :
		src_ptr_(new Eigen::MatrixXf),
		target_ptr_(new Eigen::MatrixXf)
	{
		
	}
	~SolveP2P();

	bool setData(Eigen::MatrixXf src, Eigen::MatrixXf target);


protected:
	PointDataPtr  src_ptr_;
	PointDataPtr  target_ptr_;


	
};

inline bool SolveP2P::setData(Eigen::MatrixXf src, Eigen::MatrixXf target)
{
	*src_ptr_ = src;
	*target_ptr_ = target;
	return true;
}

#pragma once
#include <flann\flann.hpp>



namespace pcl
{
	template<typename PointInT>
	class kdtree_feature
{
public:
	typedef boost::shared_ptr < flann::Index<flann::L2<float>>>  IndexPtr;
	typedef const boost::shared_ptr<pcl::PointCloud<PointInT>> PointCloudConstPtr;

	kdtree_feature();
	~kdtree_feature();

	bool setInputCloud(const  PointCloudConstPtr input);

	int searchNNearest(PointInT point, int N, std::vector<int>& idx_vector, std::vector<float>& dis_vector);
private:

	boost::shared_ptr<flann::Matrix<float>> matrix_ptr_;

	int dimension_i_;
	IndexPtr index_ptr_;
	float *trash_ptr_;
	//flann::Matrix<float> dataset;
};

	template <typename PointInT>
	kdtree_feature<PointInT>::kdtree_feature()// :index_ptr_(new flann::Index<flann::L2<float>>)
	{
	}

	template <typename PointInT>
	kdtree_feature<PointInT>::~kdtree_feature()
	{
		delete[] trash_ptr_;
	}

	template <typename PointInT>
	bool kdtree_feature<PointInT>::setInputCloud(PointCloudConstPtr input)
	{
		dimension_i_ = input->at(1).getNumberOfDimensions();

		flann::Matrix<float> datasets(new float[input->size() * dimension_i_], input->size(), dimension_i_);
		
		//matrix_ptr_ = datasets;
		for (int j(0); j < input->size();++j)
		{
			for (int k(0); k < dimension_i_;++k)
			{
				//datasets.data[j*dimension_i_ + k] = input->at(j).histogram[k];
				*(datasets.ptr() + j*dimension_i_ + k) = input->at(j).histogram[k];
			}
					                      
		}

		IndexPtr    tmp(new flann::Index<flann::L2<float>>(datasets, flann::KDTreeIndexParams(1)));
		index_ptr_ = tmp;
		index_ptr_->buildIndex();
		trash_ptr_ = datasets.ptr();

		//delete[] datasets.ptr();
		return true;
	}

	template <typename PointInT>
	int kdtree_feature<PointInT>::searchNNearest(PointInT point, 
		int N, 
		std::vector<int>& idx_vector, 
		std::vector<float>& dis_vector)
	{
		flann::Matrix<int> indices(new int[N*dimension_i_], N, dimension_i_);
		flann::Matrix<float> dists(new float[N*dimension_i_], N, dimension_i_);

		flann::Matrix<float> query(new float[dimension_i_], 1, dimension_i_);
		for (int k(0); k < dimension_i_;++k)
		{
			*(query.ptr() + k) = point.histogram[k];
		}
		index_ptr_->knnSearch(query, indices, dists, N, flann::SearchParams(-1, 0.05));
		for (int k(0); k < N;++k)
		{
			idx_vector.push_back (*(indices.ptr()+k));
			float dis(0.0);
			for (int i(0); i < dimension_i_;++i)
			{
				dis += (*(dists.ptr() + i + k*dimension_i_))*(*(dists.ptr() + i+k*dimension_i_));
			}
			dis_vector.push_back(sqrt(dis));
#ifdef _DEBUG
			//For test 
			

#endif
			//std::cout << endl;
		}
		return N;
	}
}


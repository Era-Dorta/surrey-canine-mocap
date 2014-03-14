/*
 * KNNSearch.h
 *
 *  Created on: 27 Jan 2014
 *      Author: m04701
 */

#ifndef KNNSEARCH_H_
#define KNNSEARCH_H_

#include "../../Model/PointCloud.h"
#include <flann/flann.hpp>

class KNNSearch {
public:
	KNNSearch();

	unsigned int get_checks() const;
	void set_checks(unsigned int checks);

	unsigned int get_num_kd_trees() const;
	void set_num_kd_trees(unsigned int num_kd_trees);

	bool knn_search(const PointCloudPtr& cloud, unsigned int knn,
			std::vector<std::vector<int> >& indices,
			std::vector<std::vector<float> >& dists);

private:
	void point_cloud_to_flann_matrix(const PointCloudPtr& cloud_in,
			flann::Matrix<float>& cloud_out);

	unsigned int checks;
	unsigned int num_kd_trees;
};

#endif /* KNNSEARCH_H_ */

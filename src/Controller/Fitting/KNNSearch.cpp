/*
 * KNNSearch.cpp
 *
 *  Created on: 27 Jan 2014
 *      Author: m04701
 */

#include "KNNSearch.h"

KNNSearch::KNNSearch() {
	checks = 128;
	num_kd_trees = 4;
}

unsigned int KNNSearch::get_checks() const {
	return checks;
}

void KNNSearch::set_checks(unsigned int checks) {
	this->checks = checks;
}

unsigned int KNNSearch::get_num_kd_trees() const {
	return num_kd_trees;
}

void KNNSearch::set_num_kd_trees(unsigned int num_kd_trees) {
	this->num_kd_trees = num_kd_trees;
}

bool KNNSearch::knn_search(const PointCloudPtr& cloud, unsigned int knn,
		std::vector<std::vector<int> >& indices,
		std::vector<std::vector<float> >& dists) {

	if (cloud->size() == 0 || cloud->size() - 1 < knn) {
		return false;
	}

	//Since it is a Vec3Array it has 3 dimensions
	//Data set are all the points and we want distance from every point to the
	//others so query are all the points too
	flann::Matrix<float> dataset(new float[cloud->size() * 3], cloud->size(),
			3);
	flann::Matrix<float> query(new float[cloud->size() * 3], cloud->size(), 3);

	point_cloud_to_flann_matrix(cloud, dataset);
	point_cloud_to_flann_matrix(cloud, query);

	// construct an randomised kd-tree index using 4 kd-trees
	flann::Index<flann::L2_3D<float> > index(dataset,
			flann::KDTreeIndexParams(num_kd_trees));
	index.buildIndex();

	// do a knn search, using 128 checks
	index.knnSearch(query, indices, dists, knn, flann::SearchParams(checks));

	delete[] dataset.ptr();
	delete[] query.ptr();

	return true;
}

void KNNSearch::point_cloud_to_flann_matrix(const PointCloudPtr& cloud_in,
		flann::Matrix<float>& cloud_out) {
	for (unsigned int i = 0; i < cloud_in->size(); i++) {
		cloud_out[i][0] = cloud_in->get_x(i);
		cloud_out[i][1] = cloud_in->get_y(i);
		cloud_out[i][2] = cloud_in->get_z(i);
	}
}

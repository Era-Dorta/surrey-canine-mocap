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

bool KNNSearch::knn_search(osg::ref_ptr<osg::Vec3Array> data, unsigned int knn,
		std::vector<std::vector<int> >& indices,
		std::vector<std::vector<float> >& dists) {

	if (data->size() == 0 || data->size() - 1 < knn) {
		return false;
	}

	//Since it is a Vec3Array it has 3 dimensions
	flann::Matrix<float> dataset(new float[data->size() * 3], data->size(), 3);
	flann::Matrix<float> query(new float[data->size() * 3], data->size(), 3);

	vec3Array_to_flann_matrix(data, dataset);
	vec3Array_to_flann_matrix(data, query);

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

void KNNSearch::vec3Array_to_flann_matrix(osg::ref_ptr<osg::Vec3Array> data_in,
		flann::Matrix<float>& data_out) {
	for (unsigned int i = 0; i < data_in->size(); i++) {
		data_out[i][0] = data_in->at(i).x();
		data_out[i][1] = data_in->at(i).y();
		data_out[i][2] = data_in->at(i).z();
	}
}

void KNNSearch::flann_matrix_to_vec3Array(const flann::Matrix<float>& data_in,
		osg::ref_ptr<osg::Vec3Array> data_out) {

	data_out->resize(data_in.rows * data_in.cols);

	for (unsigned int i = 0; i < data_in.rows; i++) {
		data_out->at(i).x() = data_in[i][0];
		data_out->at(i).y() = data_in[i][1];
		data_out->at(i).z() = data_in[i][2];
	}
}

void KNNSearch::flann_matrix_to_vec3Array(const flann::Matrix<int>& data_in,
		osg::ref_ptr<osg::Vec3Array> data_out) {

	data_out->resize(data_in.rows * data_in.cols);

	for (unsigned int i = 0; i < data_in.rows; i++) {
		data_out->at(i).x() = data_in[i][0];
		data_out->at(i).y() = data_in[i][1];
		data_out->at(i).z() = data_in[i][2];
	}
}

/*
 * KNNSearch.h
 *
 *  Created on: 27 Jan 2014
 *      Author: m04701
 */

#ifndef KNNSEARCH_H_
#define KNNSEARCH_H_

#include <flann/flann.hpp>
#include <osg/Array>

class KNNSearch {
public:
	KNNSearch();

	unsigned int get_checks() const;
	void set_checks(unsigned int checks);

	unsigned int get_num_kd_trees() const;
	void set_num_kd_trees(unsigned int num_kd_trees);

	bool knn_search(osg::ref_ptr<osg::Vec3Array> data, unsigned int knn,
			std::vector<std::vector<int> >& indices,
			std::vector<std::vector<float> >& dists);

private:
	void vec3Array_to_flann_matrix(osg::ref_ptr<osg::Vec3Array> data_in,
			flann::Matrix<float>& data_out);
	void flann_matrix_to_vec3Array(const flann::Matrix<float>& data_in,
			osg::ref_ptr<osg::Vec3Array> data_out);
	void flann_matrix_to_vec3Array(const flann::Matrix<int>& data_in,
			osg::ref_ptr<osg::Vec3Array> data_out);
	unsigned int checks;
	unsigned int num_kd_trees;
};

#endif /* KNNSEARCH_H_ */

/*
 * GroundTruth.h
 *
 *  Created on: 7 Mar 2014
 *      Author: m04701
 */

#ifndef GROUNDTRUTH_H_
#define GROUNDTRUTH_H_

#include "../Model/Skeleton.h"
#include <fstream>
#include <iostream>

class GroundTruth {
public:
	GroundTruth();

	void set_ground_truth_frame(const std::vector<osg::Vec3>& points,
			unsigned int frame_num);

	bool save_data(const std::string& path);

	bool load_data(const std::string& path);

	float calculate_total_error_skeleton(const SkeletonPtr skeleton);

	float calculate_frame_error_skeleton(const SkeletonPtr skeleton,
			int frame_num);

	float calculate_frame_bone_error(const SkeletonPtr skeleton,
			unsigned int bone_index, int frame);

	bool is_data_loaded() const;

private:
	void save_points(std::ofstream& out_file);

	void load_points(std::ifstream& in_file, int num_frames,
			int num_points_frame);

	float read_float(std::stringstream& ss);

	typedef std::vector<osg::Vec3> Vec3Vec;
	std::vector<Vec3Vec> points_vec;
	bool data_loaded;
};

#endif /* GROUNDTRUTH_H_ */

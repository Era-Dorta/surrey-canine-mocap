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
	GroundTruth(unsigned int max_user_points);

	void set_ground_truth_frame(const std::vector<osg::Vec3>& points,
			unsigned int frame_num);

	bool save_data(const std::string& path);

	bool load_data(const std::string& path);

	float calculate_total_error_skeleton(const SkeletonPtr skeleton);

	float calculate_frame_error_skeleton(const SkeletonPtr skeleton,
			int frame_num);

	float calculate_frame_bone_error(const SkeletonPtr skeleton,
			unsigned int bone_index, int frame);

	float calculate_root_start_frame_bone_error(const SkeletonPtr skeleton,
			int frame);

	bool is_data_loaded() const;

	const osg::Vec3& get_point(unsigned int index, unsigned int frame_num);

private:
	void save_points(std::ofstream& out_file);

	void load_points(std::ifstream& in_file, int num_frames);

	float read_float(std::stringstream& ss);

	typedef std::vector<std::vector<osg::Vec3> > Vec3DVec;

	Vec3DVec points_vec;
	bool data_loaded;
	unsigned int max_user_points;
	const osg::Vec3 empty;
};

#endif /* GROUNDTRUTH_H_ */

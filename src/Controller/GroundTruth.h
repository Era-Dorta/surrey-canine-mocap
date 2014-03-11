/*
 * GroundTruth.h
 *
 *  Created on: 7 Mar 2014
 *      Author: m04701
 */

#ifndef GROUNDTRUTH_H_
#define GROUNDTRUTH_H_

#include "../Model/Skeleton.h"

class GroundTruth {
public:
	GroundTruth();

	void set_ground_truth_frame(const std::vector<osg::Vec3>& points,
			int frame_num);

	void save_data(const std::string& path);

	void load_data(const std::string& path);

	float calculate_total_error_skeleton(const SkeletonPtr skeleton);

	float calculate_frame_error_skeleton(const SkeletonPtr skeleton,
			int frame_num);

	float calculate_frame_bone_error(const SkeletonPtr skeleton,
			unsigned int bone_index, int frame);
private:
	typedef std::vector<osg::Vec3> Vec3Vec;
	std::vector<Vec3Vec> points_vec;
};

#endif /* GROUNDTRUTH_H_ */

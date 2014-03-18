/*
 * GroundTruth.h
 *
 *  Created on: 7 Mar 2014
 *      Author: m04701
 */

#ifndef GROUNDTRUTH_H_
#define GROUNDTRUTH_H_

#include "../Model/Skeleton.h"
#include "../Model/PointCloud.h"

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

	const osg::Vec3 get_point(unsigned int index, unsigned int frame_num);

private:
	unsigned int max_user_points;
	PointCloud cloud;
	bool data_loaded;
	const osg::Vec3 empty;
};

#endif /* GROUNDTRUTH_H_ */

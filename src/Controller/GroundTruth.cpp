/*
 * GroundTruth.cpp
 *
 *  Created on: 7 Mar 2014
 *      Author: m04701
 */

#include "GroundTruth.h"

GroundTruth::GroundTruth(unsigned int max_user_points) {
	this->max_user_points = max_user_points;
	data_loaded = false;
}

void GroundTruth::set_ground_truth_frame(const std::vector<osg::Vec3>& points,
		unsigned int frame_num) {
	if (frame_num >= cloud.get_height()) {
		cloud.resize(max_user_points, frame_num + 1);
	}

	int index_offset = 0;
	for (unsigned int i = 0; i < points.size(); i++) {
		cloud.set_osg(frame_num, i + index_offset, points[i]);

		//We don't have information on bones 2, 6, 11 and 15 so
		//leave their positions at (0, 0, 0)
		if (i == 2 || i == 5 || i == 9 || i == 12) {
			index_offset++;
		}
	}
}

bool GroundTruth::save_data(const std::string& path) {
	return cloud.save_to_file(path) == 0;
}

bool GroundTruth::load_data(const std::string& path) {
	data_loaded = false;
	data_loaded = cloud.load_from_file(path) == 0;
	return data_loaded;
}

float GroundTruth::calculate_total_error_skeleton(const SkeletonPtr skeleton) {
	if (cloud.size() == 0) {
		return 0;
	}
	float skeleton_error = 0;
	int num_frames = cloud.get_height();
	for (unsigned int i = 0; i < num_frames; i++) {
		skeleton_error = calculate_frame_error_skeleton(skeleton, i);
	}

	return skeleton_error / num_frames;
}

float GroundTruth::calculate_frame_error_skeleton(const SkeletonPtr skeleton,
		int frame_num) {
	float skeleton_error = 0;
	skeleton_error = calculate_root_start_frame_bone_error(skeleton, frame_num);
	for (unsigned int i = 0; i < skeleton->get_num_bones(); i++) {
		skeleton_error = calculate_frame_bone_error(skeleton, i, frame_num);
	}

	return skeleton_error / (skeleton->get_num_bones() + 1);
}

float GroundTruth::calculate_frame_bone_error(const SkeletonPtr skeleton,
		unsigned int bone_index, int frame) {

	float bone_error = 0;

	//We do not have information on this bones so error is 0
	if (bone_index == 2 || bone_index == 6 || bone_index == 11
			|| bone_index == 15) {
		return 0;
	}

	osg::Vec3 aux = skeleton->get_node(bone_index)->get_end_bone_global_pos(
			frame);

	aux = aux - cloud.get_osg(frame, bone_index + 1);

	bone_error = aux.length();

	return bone_error;
}

float GroundTruth::calculate_root_start_frame_bone_error(
		const SkeletonPtr skeleton, int frame) {

	osg::Vec3 bone_current_pos = skeleton->get_node(0)->get_global_pos(frame);

	float bone_error = (bone_current_pos - cloud.get_osg(frame, 0)).length();

	return bone_error;
}

bool GroundTruth::is_data_loaded() const {
	return data_loaded;
}

const osg::Vec3 GroundTruth::get_point(unsigned int index,
		unsigned int frame_num) {
	if (frame_num < cloud.get_height() && index < cloud.get_width()) {
		return cloud.get_osg(frame_num, index);
	}
	return empty;
}

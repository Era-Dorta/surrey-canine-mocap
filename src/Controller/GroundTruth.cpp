/*
 * GroundTruth.cpp
 *
 *  Created on: 7 Mar 2014
 *      Author: m04701
 */

#include "GroundTruth.h"

GroundTruth::GroundTruth() {
	// TODO Auto-generated constructor stub

}

void GroundTruth::set_ground_truth_frame(const std::vector<osg::Vec3>& points,
		int frame_num) {
}

void GroundTruth::save_data(const std::string& path) {
}

void GroundTruth::load_data(const std::string& path) {
}

float GroundTruth::calculate_total_error_skeleton(const SkeletonPtr skeleton) {
}

float GroundTruth::calculate_frame_error_skeleton(const SkeletonPtr skeleton,
		int frame_num) {
}

float GroundTruth::calculate_frame_bone_error(const SkeletonPtr skeleton,
		unsigned int bone_index, int frame) {
}

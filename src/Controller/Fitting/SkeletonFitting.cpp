/*
 * SkeletonFitting.cpp
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"

SkeletonFitting::SkeletonFitting(SkeletonPtr skeleton,
		Skeletonization3DPtr skeletonization3d, const camVecT& camera_arr) :
		leg_fitter(skeleton, skeletonization3d, camera_arr), body_fitter(
				skeleton, skeletonization3d, camera_arr) {
	current_frame = -1;
	first_call = true;
	skeletonizator = skeletonization3d;
	this->skeleton = skeleton;
}

SkeletonFitting::~SkeletonFitting() {
}


//TODO Implement some form of time coherence

void SkeletonFitting::calculate_for_frame(int frame_num) {
	if (current_frame != frame_num) {
		current_frame = frame_num;
		if (first_call) {
			cloud_clusterer.init(skeletonizator->get_n_frames(),
					skeletonizator->get_d_rows(), skeletonizator->get_d_cols());
			first_call = false;
		}
		cloud = skeletonizator->get_merged_3d_projection(current_frame);
		cloud_clusterer.divide_four_sections(cloud, labels, current_frame);
		body_fitter.set_current_frame(current_frame);
		leg_fitter.set_current_frame(current_frame);
	}
}

void SkeletonFitting::fit_skeleton_to_cloud() {

	body_fitter.fit_root_position(cloud, labels);
	body_fitter.fit_head_and_back(cloud, labels);

	leg_fitter.fit_leg_position_complete(Skeleton::Front_Right, cloud, labels);
	leg_fitter.fit_leg_position_complete(Skeleton::Front_Left, cloud, labels);
	leg_fitter.fit_leg_position_complete(Skeleton::Back_Right, cloud, labels);
	leg_fitter.fit_leg_position_complete(Skeleton::Back_Left, cloud, labels);

	for (unsigned int i = 0; i < skeleton->get_num_bones(); i++) {
		skeleton->get_node(i)->optimize_rotation(current_frame);
	}
}

const std::vector<Skeleton::Skel_Leg>& SkeletonFitting::getLabels() const {
	return labels;
}

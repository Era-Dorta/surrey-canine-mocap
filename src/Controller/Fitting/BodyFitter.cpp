/*
 * BodyFitter.cpp
 *
 *  Created on: 24 Feb 2014
 *      Author: m04701
 */

#include "BodyFitter.h"

BodyFitter::BodyFitter(SkeletonPtr skeleton,
		Skeletonization3DPtr skeletonization3d, const camVecT& camera_arr) :
		CommonFitter(skeleton, skeletonization3d, camera_arr) {
}

bool BodyFitter::fit_root_position(const PointCloudPtr& cloud,
		const std::vector<Skeleton::Skel_Leg>& labels) {

	int head_index = bone_pos_finder.find_head(cloud, labels);

	if (head_index == -1) {
		return false;
	}

	osg::Vec3 translation = cloud->get_osg(head_index)
			- skeleton->get_root()->get_offset()
			- skeleton->get_root()->froset->at(current_frame);

	skeleton->translate_root(translation);

	return true;
}

int BodyFitter::fit_head_and_back(const PointCloudPtr& cloud,
		const std::vector<Skeleton::Skel_Leg>& labels) {
	int head_index = bone_pos_finder.find_head(cloud, labels);
	unsigned int bones_found = 0;

	if (head_index == -1) {
		return 0;
	}

	//Calculate positions
	//First bone
	osg::Vec3 head_pos, root_pos = cloud->get_osg(head_index);
	//Use the third camera since it gives the best head view
	const cv::Mat& cam2_bin_img = skeletonizator->get_2D_bin_frame(2,
			current_frame);
	float bone_length = skeleton->get_node(0)->get_length();
	constCamVecIte cam_ite = camera_arr.begin() + 2;

	if (!bone_pos_finder.find_first_bone_end_pos(cam2_bin_img, bone_length,
			cam_ite, current_frame, root_pos, head_pos)) {
		return 0;
	}
	bones_found++;

	//Second bone
	osg::Vec3 shoulder_pos;
	//Use the second camera for a side view
	const cv::Mat& cam1_bin_img = skeletonizator->get_2D_bin_frame(1,
			current_frame);
	bone_length = skeleton->get_node(1)->get_length();
	cam_ite = camera_arr.begin() + 1;

	//TODO If the bone finder was on looking for the position but there were
	//not enough points then infer the position using the current
	//points like how it is done with the legs
	if (bone_pos_finder.find_second_bone_end_pos(cam1_bin_img, bone_length,
			cam_ite, current_frame, head_pos, shoulder_pos)) {
		bones_found++;
	}

	//Third bone
	osg::Vec3 vertebral_end_pos;
	bone_length = skeleton->get_node(10)->get_length();

	if (bones_found == 2
			&& bone_pos_finder.find_vertebral_end_pos(cam1_bin_img, bone_length,
					cam_ite, current_frame, shoulder_pos, vertebral_end_pos)) {
		bones_found++;
	}

	//IMPORTANT NOTICE
	//Do not put the solve chain after each bone pos finder or the finding
	//will fail
	//TODO Fix above

	//Put the three bones in the calculated positions
	if (!solve_chain(0, 0, head_pos)) {
		return 0;
	}

	if (bones_found >= 2 && !solve_chain(1, 1, shoulder_pos)) {
		return 1;
	}

	if (bones_found == 3 && !solve_chain(10, 10, vertebral_end_pos)) {
		return 2;
	}

	return bones_found;
}

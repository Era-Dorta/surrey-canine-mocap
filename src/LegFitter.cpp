/*
 * LegFitter.cpp
 *
 *  Created on: 24 Feb 2014
 *      Author: m04701
 */

#include "LegFitter.h"

LegFitter::LegFitter(boost::shared_ptr<Skeleton> skeleton,
		boost::shared_ptr<Skeletonization3D> skeletonization3d,
		const camVecT& camera_arr) :
		CommonFitter(skeleton, skeletonization3d, camera_arr) {
	current_frame = 0;
}

bool LegFitter::fit_leg_position_complete(Skeleton::Skel_Leg leg,
		const osg::ref_ptr<osg::Vec3Array> cloud,
		const std::vector<Skeleton::Skel_Leg>& labels) {
	this->cloud = cloud;
	std::vector<int> leg_points_index;
	bool fit_succes;

	//TODO Change all fitting methods to return a vector of goal positions
	//and then work with that vector
	int paw_index = -1;

	//The fitting method is sensible to initialisation, then it is better
	//to first do a rough approximation

	//Put paw in lower leg point cloud position
	bool fit_simple = fit_leg_position_initialise(leg, paw_index,
			leg_points_index, labels);

	if (paw_index == -1 || leg_points_index.size() == 0) {
		return false;
	}

	//Best method
	//Paw same and mid bone going up the cloud bone length distance
	if (fit_leg_position_go_up_y(leg, paw_index, leg_points_index)) {
		return true;
	}

	//If it failed try the others
	//Paw same and mid bone in upper leg point cloud position
	fit_succes = fit_leg_position_mid_pos_in_top_leg(leg, paw_index,
			leg_points_index, labels);

	if (!fit_succes) {
		//Paw same and mid bone in arithmetic middle position
		fit_succes = fit_leg_position_half_way(leg, paw_index);
	}

	//If any of the fitting methods worked then try rotate to a better
	//position the lower two bones
	if (fit_simple || fit_succes) {
		fix_leg_second_lower_joint(leg, leg_points_index);
	}

	return fit_succes;
}

bool LegFitter::fit_leg_position_initialise(Skeleton::Skel_Leg leg,
		int& paw_index, std::vector<int>& leg_points_index,
		const std::vector<Skeleton::Skel_Leg>& labels) {

	//This initialises labels and leg_point_index
	paw_index = bone_pos_finder.find_paw(cloud, labels, leg, leg_points_index);

	if (paw_index == -1) {
		return false;
	}

	sortstruct s(this, CompMethods::comp_y);
	std::sort(leg_points_index.begin(), leg_points_index.end(), s);

	//Actualise paw index since we have modified the leg_point_index vector
	paw_index = leg_points_index.front();

	//Solve leg
	if (!solve_chain(leg - 3, leg, cloud->at(paw_index))) {
		return false;
	}
	return true;
}

bool LegFitter::fit_leg_position_go_up_y(Skeleton::Skel_Leg leg, int paw_index,
		const std::vector<int>& leg_points_index) {

	if (paw_index == -1) {
		return false;
	}

	float bone_lengths[3];
	bone_lengths[0] = skeleton->get_node(leg)->length.length();
	bone_lengths[1] = skeleton->get_node(leg - 1)->length.length();
	bone_lengths[2] = skeleton->get_node(leg - 2)->length.length();

	osg::Vec3 bone_pos[3];
	int num_valid = bone_pos_finder.find_leg_lower_3_joints_simple(cloud,
			leg_points_index, bone_lengths, bone_pos);

	if (num_valid != 3) {
		return false;
	}

	//Try put all the bones in their positions
	if (solve_leg_3_pos(leg, bone_pos[0], bone_pos[1], bone_pos[2])) {
		return true;
	}

	//Try an approximate position method
	int solved_bones = solve_leg_2_pos_approx(leg, bone_pos[1], bone_pos[2]);

	//Approximate method worked
	if (solved_bones == 2) {
		return true;
	}

	if (solved_bones == 0) {
		//Try the gradient method on the wrist bone
		if (solve_chain_keep_next_pos_gradient(leg - 3, leg - 1, bone_pos[1])) {
			solved_bones = 1;
		}
	}

	//Some method worked for the wrist bone
	//Try a constant gradient method to put the elbow bone as close to the
	//calculated position as possible
	if (solved_bones == 1) {
		solve_chain_keep_next_pos_gradient(leg - 3, leg - 2, bone_pos[2]);

		//If it fails at least we got the wrist bone is a good position
		//so return true to avoid that being undone by the other methods
		return true;
	}
	return false;
}

bool LegFitter::fit_leg_position_mid_pos_in_top_leg(Skeleton::Skel_Leg leg,
		int paw_index, const std::vector<int>& leg_points_index,
		const std::vector<Skeleton::Skel_Leg>& labels) {

	if (paw_index == -1) {
		return false;
	}
	//Put the "shoulder" at the highest point of the cloud for this leg
	int mid_position = bone_pos_finder.find_leg_upper_end_fast(
			leg_points_index);

	return fit_leg_pos_impl(leg, cloud->at(mid_position), cloud->at(paw_index));
}

bool LegFitter::fit_leg_position_half_way(Skeleton::Skel_Leg leg,
		int paw_index) {

	if (paw_index == -1) {
		return false;
	}
	int prev_bone_index = 0;

	switch (leg) {
	case Skeleton::Front_Right:
	case Skeleton::Front_Left:
		prev_bone_index = 1;
		break;
	case Skeleton::Back_Right:
	case Skeleton::Back_Left:
		prev_bone_index = 10;
		break;
	case Skeleton::Not_Limbs:
		cout << "Call to fit leg with Not_limbs" << endl;
		return false;
	}

	Node* n_bone = skeleton->get_node(prev_bone_index);
	osg::Vec3 prev_bone_position = n_bone->get_end_bone_global_pos(
			current_frame);

	//Put the "shoulder" two bones halfway from the previous bones
	//and the paw
	osg::Vec3 mid_position = (cloud->at(paw_index) + prev_bone_position) * 0.5;

	return fit_leg_pos_impl(leg, mid_position, cloud->at(paw_index));

}

bool LegFitter::fix_leg_second_lower_joint(Skeleton::Skel_Leg leg,
		const std::vector<int>& leg_points_index) {

	float max_y = skeleton->get_node(leg)->get_end_bone_global_pos(
			current_frame).y();
	float half_y = skeleton->get_node(leg)->get_global_pos(current_frame).y();
	float min_y =
			skeleton->get_node(leg - 1)->get_global_pos(current_frame).y();

	max_y = max_y - (max_y - half_y) * 0.5;
	min_y = min_y + (half_y - min_y) * 0.5;

	std::vector<int> reduced_leg_points_index;
	reduce_points_with_height(max_y, min_y, leg_points_index,
			reduced_leg_points_index);

	float best_angle = 0.0;
	int steps = 314;
	float increment = osg::PI / steps;

	osg::Matrix first_bone_old_rot, first_bone_old_trans;
	osg::Vec3 dir_vec;
	skeleton->get_matrices_for_rotate_keep_end_pos(leg, first_bone_old_rot,
			first_bone_old_trans, dir_vec);

	osg::Matrix m;
	Node* parent = skeleton->get_node(leg - 1);
	osg::Vec3 length = parent->length;
	if (parent->parent) {
		parent->parent->get_global_matrix(current_frame, m);
	} else {
		m.makeIdentity();
	}

	osg::Matrix current_m = first_bone_old_rot * first_bone_old_trans * m;
	osg::Vec3 bone_end_pos = length * current_m;

	float current_distance = calculate_sum_distance2_to_cloud(bone_end_pos,
			reduced_leg_points_index);

	//Try several rotations to find the best
	for (float angle = increment; angle < steps; angle += increment) {

		osg::Quat new_rot(angle, dir_vec);

		osg::Matrix new_m = first_bone_old_rot * osg::Matrix::rotate(new_rot)
				* first_bone_old_trans * m;
		//We only care if the bones are closer or not, so we do not
		//bother to calculate the actual distance
		bone_end_pos = length * new_m;

		//We only care if the bones are closer or not, so we do not
		//bother to calculate the actual distance
		float new_distance = calculate_sum_distance2_to_cloud(bone_end_pos,
				reduced_leg_points_index);

		if (new_distance < current_distance) {
			current_distance = new_distance;
			best_angle = angle;
		}
	}

	//Since the bones have been rotated current_angle, to put them at
	//best angle, they have to be rotated best - current
	skeleton->rotate_two_bones_keep_end_pos(leg, best_angle);
	return true;
}

bool LegFitter::fix_leg_second_lower_joint(Skeleton::Skel_Leg leg,
		const osg::Vec3& goal_pos) {

	float best_angle = 0.0;
	int steps = 314;
	float increment = osg::PI / steps;

	osg::Matrix first_bone_old_rot, first_bone_old_trans;
	osg::Vec3 dir_vec;
	skeleton->get_matrices_for_rotate_keep_end_pos(leg, first_bone_old_rot,
			first_bone_old_trans, dir_vec);

	osg::Matrix m;
	Node* parent = skeleton->get_node(leg - 1);
	osg::Vec3 length = parent->length;
	if (parent->parent) {
		parent->parent->get_global_matrix(current_frame, m);
	} else {
		m.makeIdentity();
	}

	osg::Matrix current_m = first_bone_old_rot * first_bone_old_trans * m;
	osg::Vec3 bone_end_pos = length * current_m;

	float current_distance = (bone_end_pos - goal_pos).length2();

	//Try several rotations to find the best
	for (float angle = increment; angle < steps; angle += increment) {

		osg::Quat new_rot(angle, dir_vec);

		osg::Matrix new_m = first_bone_old_rot * osg::Matrix::rotate(new_rot)
				* first_bone_old_trans * m;
		//We only care if the bones are closer or not, so we do not
		//bother to calculate the actual distance
		bone_end_pos = length * new_m;
		float new_distance = (bone_end_pos - goal_pos).length2();

		if (new_distance < current_distance) {
			current_distance = new_distance;
			best_angle = angle;
		}
	}

	if (best_angle != 0.0) {
		skeleton->rotate_two_bones_keep_end_pos(leg, best_angle);
	}
	return true;
}

bool LegFitter::fit_leg_pos_impl(Skeleton::Skel_Leg leg,
		const osg::Vec3& middle_position, const osg::Vec3& paw_position) {
	//Save top two bones rotations
	osg::Quat prev0, prev1;
	prev0 = skeleton->get_node(leg - 3)->quat_arr.at(current_frame);
	prev1 = skeleton->get_node(leg - 2)->quat_arr.at(current_frame);

	//Solve for "shoulder" two bones
	if (!solve_chain(leg - 3, leg - 2, middle_position)) {
		return false;
	}

	//Solve for paw and parent bone
	if (!solve_chain(leg - 1, leg, paw_position)) {
		//If second pair failed then restore previous rotations
		skeleton->get_node(leg - 3)->quat_arr.at(current_frame) = prev0;
		skeleton->get_node(leg - 2)->quat_arr.at(current_frame) = prev1;
		return false;
	}
	return true;
}

float LegFitter::calculate_sum_distance2_to_cloud(const osg::Vec3& bone_end_pos,
		const std::vector<int>& leg_points_index) {
	float distance = 0.0;
	//This methods returns the square of the sum of the distances of the
	//leg points to the bone end position
	std::vector<int>::const_iterator i = leg_points_index.begin();
	for (; i != leg_points_index.end(); ++i) {
		distance += (bone_end_pos - cloud->at(*i)).length2();
	}
	return distance;
}

void LegFitter::reduce_points_with_height(float max_y, float min_y,
		const std::vector<int>& leg_points_index,
		std::vector<int>& new_leg_points_index) {
	new_leg_points_index.clear();

	std::vector<int>::const_iterator i = leg_points_index.begin();
	for (; i != leg_points_index.end(); ++i) {
		if (cloud->at(*i).y() < max_y && cloud->at(*i).y() > min_y) {
			new_leg_points_index.push_back(*i);
		}
	}
}

//Pos0 is paw position, and pos1 and pos2 of the respective parent bones
bool LegFitter::solve_leg_3_pos(Skeleton::Skel_Leg leg, const osg::Vec3& pos0,
		const osg::Vec3& pos1, const osg::Vec3& pos2) {
	osg::Quat prev0, prev1, prev2, prev3;
	prev0 = skeleton->get_node(leg)->quat_arr.at(current_frame);
	prev1 = skeleton->get_node(leg - 1)->quat_arr.at(current_frame);
	prev2 = skeleton->get_node(leg - 2)->quat_arr.at(current_frame);
	prev3 = skeleton->get_node(leg - 3)->quat_arr.at(current_frame);

	//Shoulder and elbow
	if (!solve_chain(leg - 3, leg - 2, pos2)) {
		skeleton->get_node(leg - 2)->quat_arr.at(current_frame) = prev2;
		skeleton->get_node(leg - 3)->quat_arr.at(current_frame) = prev3;
		return false;
	}

	//Wrist
	if (!solve_chain(leg - 1, leg - 1, pos1)) {
		skeleton->get_node(leg - 1)->quat_arr.at(current_frame) = prev1;
		skeleton->get_node(leg - 2)->quat_arr.at(current_frame) = prev2;
		skeleton->get_node(leg - 3)->quat_arr.at(current_frame) = prev3;
		return false;
	}

	//Paw
	if (!solve_chain(leg, leg, pos0)) {
		skeleton->get_node(leg)->quat_arr.at(current_frame) = prev0;
		skeleton->get_node(leg - 1)->quat_arr.at(current_frame) = prev1;
		skeleton->get_node(leg - 2)->quat_arr.at(current_frame) = prev2;
		skeleton->get_node(leg - 3)->quat_arr.at(current_frame) = prev3;
		return false;
	}
	return true;
}

int LegFitter::solve_leg_2_pos_approx(Skeleton::Skel_Leg leg,
		const osg::Vec3& pos1, const osg::Vec3& pos2) {
	osg::Quat prev1, prev2, prev3;
	prev1 = skeleton->get_node(leg - 1)->quat_arr.at(current_frame);
	prev2 = skeleton->get_node(leg - 2)->quat_arr.at(current_frame);
	prev3 = skeleton->get_node(leg - 3)->quat_arr.at(current_frame);

	//Wrist
	if (!solve_chain_keep_next_pos(leg - 3, leg - 1, pos1)) {
		skeleton->get_node(leg - 1)->quat_arr.at(current_frame) = prev1;
		skeleton->get_node(leg - 2)->quat_arr.at(current_frame) = prev2;
		skeleton->get_node(leg - 3)->quat_arr.at(current_frame) = prev3;
		return 0;
	}

	//Shoulder and elbow
	if (!solve_chain_keep_next_pos(leg - 3, leg - 2, pos2)) {
		return 1;
	}

	return 2;
}

bool LegFitter::solve_leg_upper_pos_approx(Skeleton::Skel_Leg leg,
		const osg::Vec3& pos) {
	osg::Quat prev2, prev3;
	prev2 = skeleton->get_node(leg - 2)->quat_arr.at(current_frame);
	prev3 = skeleton->get_node(leg - 3)->quat_arr.at(current_frame);

	//Shoulder and elbow
	if (!solve_chain_keep_next_pos_ignore_res(leg - 3, leg - 1, pos)) {
		skeleton->get_node(leg - 2)->quat_arr.at(current_frame) = prev2;
		skeleton->get_node(leg - 3)->quat_arr.at(current_frame) = prev3;
		return false;
	}

	return true;
}

/*
 * SkeletonFitting.cpp
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"

SkeletonFitting::SkeletonFitting(boost::shared_ptr<Skeleton> skeleton_,
		boost::shared_ptr<Skeletonization3D> skeletonization3d,
		const camVecT& camera_arr_) :
		camera_arr(camera_arr_) {
	move_joint_max_dist = 0;
	error_threshold = 0.005;
	current_frame = -1;
	n_frames = 0;
	skeleton = skeleton_;
	skeletonizator = skeletonization3d;
	first_call = true;
}

SkeletonFitting::~SkeletonFitting() {
}

//void SkeletonFitting::fit_skeleton_into_cloud(Skeleton& skeleton,
//		osg::ref_ptr<osg::Vec3Array> cloud) {

//TODO
//Select all points that are closer than a threshold to the line formed by
//a pair of joints, care with points far away from the joints, case the line
//cuts other points in the skeleton->

//How to get the points???
//Segment in boxes first???

//Call fitLine with those points and the line formed by the joints
//New joints shall be the closest points of the new line to the previous
//joints

//fitLine(InputArray points, OutputArray line, int distType, double param, double reps, double aeps)
//points – Input vector of 2D or 3D points, stored in std::vector<> or Mat.
//line – Output line parameters. In case of 2D fitting, it should be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
//distType – Distance used by the M-estimator (see the discussion below).
//param – Numerical parameter ( C ) for some types of distances. If it is 0, an optimal value is chosen.
//reps – Sufficient accuracy for the radius (distance between the coordinate origin and the line).
//aeps – Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.

//}

//void SkeletonFitting::fit_skeleton_with_prev_nex_frame(Skeleton& skeleton,
//	int frame) {
//TODO
//Calculate distance to joint in next and previous frame
//Calculate vectors of movement to positions in next and previous frame
//Add them
//If the are not 0 in any dimension, it means that if we move in that dimension
//a certain amount, then distance to prev and next should be reduced.
//Iterative??? How much to move???
//}

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
		int head_index = bone_pos_finder.find_head(cloud, labels);
		cloud_clusterer.refine_four_sections_division(cloud, labels,
				current_frame, head_index);
	}
}

void SkeletonFitting::fit_skeleton_to_cloud() {
	fit_root_position();
	fit_head_and_back();
	fit_leg_position_complete(Skeleton::Front_Right);
	fit_leg_position_complete(Skeleton::Front_Left);
	fit_leg_position_complete(Skeleton::Back_Right);
	fit_leg_position_complete(Skeleton::Back_Left);

	for (unsigned int i = 0; i < skeleton->get_num_bones(); i++) {
		skeleton->get_node(i)->set_y_rotation_perpendicular_to_next_bone(
				current_frame);
	}
}

bool SkeletonFitting::fit_root_position() {
	int head_index = bone_pos_finder.find_head(cloud, labels);

	if (head_index == -1) {
		return false;
	}
	osg::Vec3 translation = cloud->at(head_index) - skeleton->get_root()->offset
			- skeleton->get_root()->froset->at(current_frame);
	skeleton->translate_root(translation);
	return true;

}

bool SkeletonFitting::fit_head_and_back() {

	int head_index = bone_pos_finder.find_head(cloud, labels);

	if (head_index == -1) {
		return false;
	}

	//Calculate positions
	//First bone
	osg::Vec3 head_pos, root_pos = cloud->at(head_index);
	//Use the third camera since it gives the best head view
	const cv::Mat& cam2_bin_img = skeletonizator->get_2D_bin_frame(2,
			current_frame);
	float bone_length = skeleton->get_node(0)->length.length();
	constCamVecIte cam_ite = camera_arr.begin() + 2;

	if (!bone_pos_finder.find_first_bone_end_pos(cam2_bin_img, bone_length,
			cam_ite, current_frame, root_pos, head_pos)) {
		return false;
	}
	refine_goal_position(head_pos, root_pos, bone_length);

	//Second bone
	osg::Vec3 shoulder_pos;
	//Use the second camera for a side view
	const cv::Mat& cam1_bin_img = skeletonizator->get_2D_bin_frame(1,
			current_frame);
	bone_length = skeleton->get_node(1)->length.length();
	cam_ite = camera_arr.begin() + 1;

	if (!bone_pos_finder.find_second_bone_end_pos(cam1_bin_img, bone_length,
			cam_ite, current_frame, head_pos, shoulder_pos)) {
		return false;
	}

	refine_goal_position(shoulder_pos, head_pos, bone_length);

	//Third bone
	osg::Vec3 vertebral_back_pos;
	bone_length = skeleton->get_node(10)->length.length();

	if (!bone_pos_finder.find_vertebral_end_pos(cam1_bin_img, bone_length,
			cam_ite, current_frame, shoulder_pos, vertebral_back_pos)) {
		return false;
	}

	refine_goal_position(vertebral_back_pos, shoulder_pos, bone_length);

	//Use inverse kinematics to fit bones into positions
	if (!solve_chain(0, 0, head_pos)) {
		return false;
	}

	//Use inverse kinematics to fit bones into positions
	if (!solve_chain(1, 1, shoulder_pos)) {
		return false;
	}

	if (!solve_chain(10, 10, vertebral_back_pos)) {
		return false;
	}
	return true;
}

bool SkeletonFitting::fit_leg_position_complete(Skeleton::Skel_Leg leg) {
	std::vector<int> leg_points_index;
	bool fit_succes;

	//TODO Change all fitting methods to return a vector of goal positions
	//and then work with that vector
	int paw_index = bone_pos_finder.find_paw(cloud, labels, leg,
			leg_points_index);

	//The fitting method is sensible to initialisation, then it is better
	//to first do a rough approximation

	//Put paw in lower leg point cloud position
	bool fit_simple = fit_leg_position_simple(leg, paw_index, leg_points_index);

	//Best method
	//Paw same and mid bone going up the cloud bone length distance
	std::vector<osg::Vec3> joint_positions;
	if (fit_leg_position_go_up_y(leg, paw_index, leg_points_index,
			joint_positions)) {
		return true;
	}

	//If it failed try the others
	//Paw same and mid bone in upper leg point cloud position
	fit_succes = fit_leg_position_mid_pos_in_top_leg(leg, paw_index,
			leg_points_index);

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

bool SkeletonFitting::fit_leg_position_go_up_y(Skeleton::Skel_Leg leg,
		int paw_index, std::vector<int>& leg_points_index,
		std::vector<osg::Vec3>& joint_positions) {

	joint_positions.clear();

	if (paw_index == -1) {
		return false;
	}

	//TODO Not sure if this should be here or in some other place
	//Since we are going to go up the leg better to have all the points ordered
	//along the y axis
	sortstruct s(this, CloudClusterer::comp_y);
	std::sort(leg_points_index.begin(), leg_points_index.end(), s);

	unsigned int bones_per_leg = 3;
	joint_positions.push_back(cloud->at(paw_index));
	//TODO Much more efficient to have the iterate backwards

	std::vector<int>::iterator j = leg_points_index.begin();
	unsigned int i = 1;
	bool continue_shearch = true;
	osg::Vec3 bone_start_pos = joint_positions[0];
	while (i < bones_per_leg && continue_shearch) {

		float bone_length = skeleton->get_node(leg - i + 1)->length.length();
		//Set bone length to paw_index
		//Go up bone length
		bool not_bone_length = true;

		while (not_bone_length && j != leg_points_index.end()) {

			float current_length = (bone_start_pos - cloud->at(*j)).length();
			if (current_length >= bone_length) {
				not_bone_length = false;
			} else {
				j++;
			}
		}

		if (not_bone_length == false) {
			if (joint_positions.size() < 3) {
				bone_start_pos = cloud->at(*j);
				//Make sure position is reachable
				refine_start_position(bone_start_pos, joint_positions.back(),
						bone_length);

				joint_positions.push_back(bone_start_pos);
			} else {
				//All bones positions have been found
				continue_shearch = false;
			}
		} else {
			//We run out of points
			continue_shearch = false;
		}
		i++;
	}

	if (joint_positions.size() >= bones_per_leg) {
		//TODO When calculating positions check if they are in reachable
		//if they are not recalculated using bone length and correct all
		//descends positions.
		return solve_leg_3_pos(leg, joint_positions[0], joint_positions[1],
				joint_positions[2]);
	} else {
		return false;
	}

}

bool SkeletonFitting::fit_leg_position_simple(Skeleton::Skel_Leg leg) {
	std::vector<int> leg_points_index;

	int paw_index = bone_pos_finder.find_paw(cloud, labels, leg,
			leg_points_index);

	return fit_leg_position_simple(leg, paw_index, leg_points_index);
}

bool SkeletonFitting::fit_leg_position_simple(Skeleton::Skel_Leg leg,
		int paw_index, std::vector<int>& leg_points_index) {

	if (paw_index == -1) {
		return false;
	}

	//Solve leg
	if (!solve_chain(leg - 3, leg, cloud->at(paw_index))) {
		return false;
	}
	return true;
}

bool SkeletonFitting::fit_leg_position_mid_pos_in_top_leg(
		Skeleton::Skel_Leg leg, int& paw_index) {

	std::vector<int> leg_points_index;
	return fit_leg_position_mid_pos_in_top_leg(leg, paw_index, leg_points_index);
}

bool SkeletonFitting::fit_leg_position_mid_pos_in_top_leg(
		Skeleton::Skel_Leg leg, int& paw_index,
		std::vector<int>& leg_points_index) {
	paw_index = bone_pos_finder.find_paw(cloud, labels, leg, leg_points_index);

	if (paw_index == -1) {
		return false;
	}
	//Put the "shoulder" at the highest point of the cloud for this leg
	int mid_position = bone_pos_finder.find_leg_upper_end(cloud, labels, leg,
			leg_points_index);

	return fit_leg_pos_impl(leg, cloud->at(mid_position), cloud->at(paw_index));
}

bool SkeletonFitting::fit_leg_position_half_way(Skeleton::Skel_Leg leg,
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

bool SkeletonFitting::fix_leg_second_lower_joint(Skeleton::Skel_Leg leg,
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

bool SkeletonFitting::fix_leg_second_lower_joint(Skeleton::Skel_Leg leg,
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

bool SkeletonFitting::fit_leg_pos_impl(Skeleton::Skel_Leg leg,
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

const std::vector<Skeleton::Skel_Leg>& SkeletonFitting::getLabels() const {
	return labels;
}

osg::Vec3 SkeletonFitting::get_paw(Skeleton::Skel_Leg leg) {
	std::vector<int> leg_points_index;
	int index = bone_pos_finder.find_paw(cloud, labels, leg, leg_points_index);

	if (index != -1) {
		return cloud->at(index);
	} else {
		return osg::Vec3();
	}
}

bool SkeletonFitting::solve_chain(int root_bone, int end_bone,
		const osg::Vec3& position) {

	ik_solver.start_chain();
	std::vector<int> indices;

	//Create vector of indices since root_bone can be bigger than end_bone
	if (root_bone - end_bone <= 0) {
		for (int i = root_bone; i <= end_bone; i++) {
			indices.push_back(i);
		}
	} else {
		for (int i = root_bone; i >= end_bone; i--) {
			indices.push_back(i);
		}
	}

	//Insert bones in ik_solver
	for (unsigned int i = 0; i < indices.size(); i++) {
		Node * node = skeleton->get_node(indices[i]);
		float3 offset = make_float3(node->length._v);
		osg::Quat q = node->quat_arr.at(current_frame);
		float4 rot = make_float4(q.x(), q.y(), q.z(), q.w());
		ik_solver.add_bone_to_chain(offset, rot);
	}

	//Calculate in root_bone coordinate system where is the goal position
	osg::Matrix m;
	calculate_bone_world_matrix_origin(m, skeleton->get_node(root_bone));
	osg::Vec3 goal_position = position * m;

	//Solve and update the rotations
	if (ik_solver.solve_chain(make_float3(goal_position._v))) {
		int j = 0;
		for (unsigned int i = 0; i < indices.size(); i++) {
			Node * node = skeleton->get_node(indices[i]);
			float4 new_rot;
			ik_solver.get_rotation_joint(j, new_rot);
			node->quat_arr.at(current_frame).set(new_rot.x, new_rot.y,
					new_rot.z, new_rot.w);
			j++;
		}
		return true;
	} else {
		return false;
	}
}

void SkeletonFitting::calculate_bone_world_matrix_origin(osg::Matrix& matrix,
		const Node* const node) {

	if (node->parent) {
		node->parent->get_global_matrix(current_frame, matrix);
	}

	matrix = osg::Matrix::translate(
			node->offset + node->froset->at(current_frame)) * matrix;
	matrix = osg::Matrix::inverse(matrix);
}

void SkeletonFitting::refine_goal_position(osg::Vec3& end_position,
		const osg::Vec3& base_position, float length) {
	//Recalculate bone goal position using its length so we are sure it can
	//be reached
	osg::Vec3 pos_direction = (end_position - base_position);
	pos_direction.normalize();
	end_position = base_position + pos_direction * length;
}

void SkeletonFitting::refine_start_position(osg::Vec3& start_position,
		const osg::Vec3& end_position, float length) {
	//Recalculate bone start position using its length so we are sure it can
	//be reached
	osg::Vec3 pos_direction = (start_position - end_position);
	pos_direction.normalize();
	start_position = end_position + pos_direction * length;
}

float SkeletonFitting::calculate_sum_distance2_to_cloud(
		const osg::Vec3& bone_end_pos,
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

void SkeletonFitting::reduce_points_with_height(float max_y, float min_y,
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
bool SkeletonFitting::solve_leg_3_pos(Skeleton::Skel_Leg leg,
		const osg::Vec3& pos0, const osg::Vec3& pos1, const osg::Vec3& pos2) {
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

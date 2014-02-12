/*
 * SkeletonFitting.cpp
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"
#include "DebugUtil.h"

SkeletonFitting::SkeletonFitting(boost::shared_ptr<Skeleton> skeleton_,
		boost::shared_ptr<Skeletonization3D> skeletonization3d,
		const camVecT& camera_arr_) :
		camera_arr(camera_arr_) {
	move_joint_max_dist = 0;
	error_threshold = 0.005;
	current_frame = -1;
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

	if (!fit_leg_position_mid_pos_in_top_leg(Skeleton::Front_Right)) {
		fit_leg_position_simple(Skeleton::Front_Right);
	}
	if (!fit_leg_position_mid_pos_in_top_leg(Skeleton::Front_Left)) {
		fit_leg_position_simple(Skeleton::Front_Left);
	}
	if (!fit_leg_position_mid_pos_in_top_leg(Skeleton::Back_Right)) {
		fit_leg_position_simple(Skeleton::Back_Right);
	}
	if (!fit_leg_position_mid_pos_in_top_leg(Skeleton::Back_Left)) {
		fit_leg_position_simple(Skeleton::Back_Left);
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

bool SkeletonFitting::fit_leg_position(Skeleton::Skel_Leg leg) {
	std::vector<int> leg_points_index, joint_positions_index;

	//TODO find_paw initialises leg_points_index, should have another method
	//to do this

	int paw_index = bone_pos_finder.find_paw(cloud, labels, leg,
			leg_points_index);

	if (paw_index == -1) {
		return false;
	}

	//TODO Not sure if this should be here or in some other place
	//Since we are going to go up the leg better to have all the points ordered
	//along the y axis
	sortstruct s(this, CloudClusterer::comp_y);
	std::sort(leg_points_index.begin(), leg_points_index.end(), s);

	int bones_per_leg = 4;
	joint_positions_index.resize(bones_per_leg);

	joint_positions_index[0] = paw_index;
	//TODO Much more efficient to have the iterate backwards

	std::vector<int>::iterator j = leg_points_index.begin();
	for (int i = 1; i < bones_per_leg; i++) {

		float bone_length = skeleton->get_node(leg - i + 1)->length.length();
		//Set bone length to paw_index
		//Go up bone length
		bool not_bone_length = true;

		while (not_bone_length && j != leg_points_index.end()) {

			float current_length = (cloud->at(joint_positions_index[i - 1])
					- cloud->at(*j)).length();
			if (current_length >= bone_length) {
				not_bone_length = false;
			} else {
				j++;
			}
		}

		joint_positions_index[i] = *j;
	}

	return fit_leg_pos_impl(leg, cloud->at(joint_positions_index[2]),
			cloud->at(joint_positions_index[0]));

}

bool SkeletonFitting::fit_leg_position_simple(Skeleton::Skel_Leg leg) {
	std::vector<int> leg_points_index;

	int paw_index = bone_pos_finder.find_paw(cloud, labels, leg,
			leg_points_index);

	if (paw_index == -1) {
		return false;
	}

	//Solve leg
	if (!solve_chain(leg - 3, leg, cloud->at(paw_index))) {
		cout << "Failed leg fitting simple" << endl;
		return false;
	}
	return true;
}

bool SkeletonFitting::fit_leg_position_mid_pos_in_top_leg(
		Skeleton::Skel_Leg leg) {
	std::vector<int> leg_points_index;

	int paw_index = bone_pos_finder.find_paw(cloud, labels, leg,
			leg_points_index);

	if (paw_index == -1) {
		return false;
	}
	//Put the "shoulder" at the highest point of the cloud for this leg
	int mid_position = bone_pos_finder.find_leg_upper_end(cloud, labels, leg,
			leg_points_index);

	return fit_leg_pos_impl(leg, cloud->at(mid_position), cloud->at(paw_index));
}

bool SkeletonFitting::fit_leg_pos_impl(Skeleton::Skel_Leg leg,
		const osg::Vec3& middle_position, const osg::Vec3& paw_position) {
	//Solve for "shoulder" two bones
	if (!solve_chain(leg - 3, leg - 2, middle_position)) {
		cout << "Failed leg fitting the first pair" << endl;
		return false;
	}

	//Solve for paw and parent bone
	if (!solve_chain(leg - 1, leg, paw_position)) {
		cout << "Failed leg fitting the second pair" << endl;
		return false;
	}
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
		cout << "Vertebral front fit fail" << endl;
		return false;
	}

	//Use inverse kinematics to fit bones into positions
	if (!solve_chain(1, 1, shoulder_pos)) {
		cout << "Vertebral front fit fail" << endl;
		return false;
	}

	if (!solve_chain(10, 10, vertebral_back_pos)) {
		cout << "Vertebral back fit fail" << endl;
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

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
		int head_index = find_head();
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
	int head_index = find_head();

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

	int paw_index = find_paw(leg, leg_points_index);

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

	int paw_index = find_paw(leg, leg_points_index);

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

	int paw_index = find_paw(leg, leg_points_index);

	if (paw_index == -1) {
		return false;
	}
	//Put the "shoulder" at the highest point of the cloud for this leg
	int mid_position = find_leg_upper_end(leg, leg_points_index);

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

	int head_index = find_head();

	if (head_index == -1) {
		return false;
	}

	//Calculate positions
	//First bone
	osg::Vec3 head_pos, root_pos = cloud->at(head_index);
	if (!find_first_bone_end_pos(root_pos, head_pos)) {
		return false;
	}
	float bone_length = skeleton->get_node(0)->length.length();
	refine_goal_position(head_pos, root_pos, bone_length);

	//Second bone
	osg::Vec3 shoulder_pos;
	if (!find_second_bone_end_pos(head_pos, shoulder_pos)) {
		return false;
	}
	bone_length = skeleton->get_node(1)->length.length();
	refine_goal_position(shoulder_pos, head_pos, bone_length);

	//Third bone
	osg::Vec3 vertebral_back_pos;
	if (!find_vertebral_end_pos(shoulder_pos, vertebral_back_pos)) {
		return false;
	}
	bone_length = skeleton->get_node(10)->length.length();
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
	int index = find_paw(leg, leg_points_index);

	if (index != -1) {
		return cloud->at(index);
	} else {
		return osg::Vec3();
	}
}

int SkeletonFitting::find_head() {
	int index = -1;

	if (cloud->size() > 4) {
		float max_x = -FLT_MAX;
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] == Skeleton::Not_Limbs && max_x < cloud->at(i).x()) {
				max_x = cloud->at(i).x();
				index = i;
			}
		}
	}
	return index;
}

int SkeletonFitting::find_paw(Skeleton::Skel_Leg leg,
		std::vector<int>& leg_points_index) {
	leg_points_index.clear();

	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (labels[i] == leg) {
			leg_points_index.push_back(i);
		}
	}

	if (leg_points_index.size() > 0) {
		float max_y = cloud->at(leg_points_index.front()).y();
		int index = leg_points_index.front();
		for (unsigned int i = 0; i < leg_points_index.size(); i++) {
			if (max_y < cloud->at(leg_points_index[i]).y()) {
				max_y = cloud->at(leg_points_index[i]).y();
				index = leg_points_index[i];
			}
		}
		return index;
	} else {
		return -1;
	}
}

int SkeletonFitting::find_leg_upper_end(Skeleton::Skel_Leg leg,
		std::vector<int>& leg_points_index) {
	leg_points_index.clear();

	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (labels[i] == leg) {
			leg_points_index.push_back(i);
		}
	}

	if (leg_points_index.size() > 0) {
		float min_y = cloud->at(leg_points_index.front()).y();
		int index = leg_points_index.front();
		for (unsigned int i = 0; i < leg_points_index.size(); i++) {
			if (min_y > cloud->at(leg_points_index[i]).y()) {
				min_y = cloud->at(leg_points_index[i]).y();
				index = leg_points_index[i];
			}
		}
		return index;
	} else {
		return -1;
	}
}

bool SkeletonFitting::find_first_bone_end_pos(const osg::Vec3& root_pos,
		osg::Vec3& head_pos) {
	//TODO Could also do this by having a complete cloud point
	//of the dog and going up and left (-y, -x) until distance was
	//reached, in any case a complete cloud could be useful

	float3 root_pos3 = make_float3(root_pos.x(), root_pos.y(), root_pos.z());

	//TODO SUPER IMPORTANT DO NOT DO THIS ON CAMERA BASED, DO A SUPER
	//CAMERA A PROJECT ALL POINTS TO AXES, on find second bone too
	//To be able to do this, I have to take the binary images from each camera
	//project them to 3D, and them project the resulting cloud to 2D using the
	//projections methods in this class

	//Use the third camera since it gives the best head view
	const cv::Mat& cam2_bin_img = skeletonizator->get_2D_bin_frame(2,
			current_frame);

	int row = 0, col = 0;

	float3 start_point = Projections::get_2d_projection(root_pos,
			camera_arr.begin() + 2);
	row = start_point.y;
	col = start_point.x;

	float bone_length = skeleton->get_node(0)->length.length();

	//The point could be projected on a position this camera does not have
	//information on. So find the closest point and continue from there
	if (cam2_bin_img.at<uchar>(row, col) != 255) {
		if (!PixelSearch::get_nearest_white_pixel(cam2_bin_img, row, col, row,
				col)) {
			cout << "Head error not recoverable" << endl;
			return false;
		}
	}

	bool not_bone_length = true;
	float3 aux_point;
	while (not_bone_length) {
		if (PixelSearch::get_top_left_white_pixel(cam2_bin_img, row, col, row,
				col)) {
			aux_point = Projections::get_3d_projection(row, col,
					camera_arr.begin() + 2, current_frame);
			float current_length = length(root_pos3 - aux_point);
			if (current_length >= bone_length) {
				not_bone_length = false;
			}
		} else {
			cout << "Error calculating first bone position" << endl;
			return false;
		}
	}
	head_pos.set(aux_point.x, aux_point.y, aux_point.z);
	return true;
}

bool SkeletonFitting::find_second_bone_end_pos(const osg::Vec3& head_pos,
		osg::Vec3& shoulder_pos) {
	//Use the second camera for a side view
	const cv::Mat& cam1_bin_img = skeletonizator->get_2D_bin_frame(1,
			current_frame);

	float3 start_point = Projections::get_2d_projection(head_pos,
			camera_arr.begin() + 1);

	int row = start_point.y;
	int col = start_point.x;

	if (cam1_bin_img.at<uchar>(row, col) != 255) {
		if (!PixelSearch::get_nearest_white_pixel(cam1_bin_img, row, col, row,
				col)) {
			cout << "Could not locate head end position on second camera"
					<< endl;
			return false;
		}
	}

	float bone_length = skeleton->get_node(1)->length.length();

	bool not_bone_length = true;
	float3 aux_point, head3 = make_float3(head_pos.x(), head_pos.y(),
			head_pos.z());
	while (not_bone_length) {
		if (PixelSearch::get_top_left_white_pixel(cam1_bin_img, row, col, row,
				col)) {
			aux_point = Projections::get_3d_projection(row, col,
					camera_arr.begin() + 1, current_frame);
			float current_length = length(head3 - aux_point);
			if (current_length >= bone_length) {
				not_bone_length = false;
			}
		} else {
			if (!unstuck_go_down(cam1_bin_img, row, col, row, col)) {
				cout << "Error calculating second bone position" << endl;
				return false;
			}
		}
	}
	shoulder_pos.set(aux_point.x, aux_point.y, aux_point.z);
	return true;
}

bool SkeletonFitting::find_vertebral_end_pos(const osg::Vec3& shoulder_pos,
		osg::Vec3& vertebral_end_pos) {
	//Use the second camera for a side view
	const cv::Mat& cam1_bin_img = skeletonizator->get_2D_bin_frame(1,
			current_frame);

	float3 start_point = Projections::get_2d_projection(shoulder_pos,
			camera_arr.begin() + 1);

	int row = start_point.y;
	int col = start_point.x;

	if (cam1_bin_img.at<uchar>(row, col) != 255) {
		if (!PixelSearch::get_nearest_white_pixel(cam1_bin_img, row, col, row,
				col)) {
			cout << "Could not locate shoulder end position on second camera"
					<< endl;
			return false;
		}
	}

	//Vertebral bone is index 10
	float bone_length = skeleton->get_node(10)->length.length();

	bool not_bone_length = true;
	float3 aux_point, head3 = make_float3(shoulder_pos.x(), shoulder_pos.y(),
			shoulder_pos.z());
	while (not_bone_length) {
		if (PixelSearch::get_top_left_white_pixel(cam1_bin_img, row, col, row,
				col)) {
			aux_point = Projections::get_3d_projection(row, col,
					camera_arr.begin() + 1, current_frame);
			float current_length = length(head3 - aux_point);
			if (current_length >= bone_length) {
				not_bone_length = false;
			}
		} else {
			if (!unstuck_go_down(cam1_bin_img, row, col, row, col)) {
				cout << "Error calculating third bone position" << endl;
				return false;
			}
		}
	}
	vertebral_end_pos.set(aux_point.x, aux_point.y, aux_point.z);
	return true;
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

bool SkeletonFitting::are_equal(const osg::Vec3& v0, const osg::Vec3& v1) {
	if (v0.x() + error_threshold >= v1.x() && v0.x() - error_threshold <= v1.x()
			&& v0.y() + error_threshold >= v1.y()
			&& v0.y() - error_threshold <= v1.y()
			&& v0.z() + error_threshold >= v1.z()
			&& v0.z() - error_threshold <= v1.z()) {
		return true;
	} else {
		return false;
	}
}

bool SkeletonFitting::check_bone_index(int bone0, int bone1) {
	if (bone0 >= 0 && (unsigned) bone0 < skeleton->get_num_bones() && bone1 >= 0
			&& (unsigned) bone1 < skeleton->get_num_bones()) {
		return true;
	} else {
		return false;
	}
}

bool SkeletonFitting::unstuck_go_down(const cv::Mat& img, int i_row, int i_col,
		int &res_row, int &res_col) {
	i_col--;
	if (i_col < 0) {
		return false;
	}

	while (i_row < img.rows && (int) img.at<uchar>(i_row, i_col) != 255) {
		i_row++;
	}

	if (i_row < img.rows && (int) img.at<uchar>(i_row, i_col) == 255) {
		res_row = i_row;
		res_col = i_col;
		return true;
	}

	return false;
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

void SkeletonFitting::get_y_z_front_projection(Skeleton::Skel_Leg leg,
		cv::Mat& out_img, const osg::Vec3& trans) {
	//We want a front view so in world axes is vectors
	//x = [0,0,1]
	//y = [0,1,0]
	//z = [-1,0,0]
	//For the projection position the head position is used,
	//but for the legs to be the centre of the projection
	//an offset is needed
	float4x4 invT(0.0);
	invT[0 * 4 + 2] = -1;
	invT[1 * 4 + 1] = 1;
	invT[2 * 4 + 0] = 1;
	invT[3 * 4 + 0] = trans.z() + 0.05;
	invT[3 * 4 + 1] = trans.y() + 0.0;
	invT[3 * 4 + 2] = -(trans.x() - 0.3);
	invT[3 * 4 + 3] = 1;

	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (labels[i] == leg) {
			float3 point2d = Projections::get_2d_projection(cloud->at(i), invT);
			if (point2d.y >= 0 && point2d.y < out_img.rows && point2d.x >= 0
					&& point2d.x < out_img.cols)
				out_img.at<uchar>(point2d.y, point2d.x) = 255;
		}
	}
}

void SkeletonFitting::get_x_y_side_projection(Skeleton::Skel_Leg leg,
		cv::Mat& out_img, const osg::Vec3& trans) {
	//We want a side view, so no rotation is needed
	//x = [1,0,0]
	//y = [0,1,0]
	//z = [0,0,1]
	//For the projection position the head position is used,
	//but for the legs to be the centre of the projection
	//an offset is needed
	float4x4 invT(0.0);
	invT[0 * 4 + 0] = 1;
	invT[1 * 4 + 1] = 1;
	invT[2 * 4 + 2] = 1;
	invT[3 * 4 + 0] = trans.x() + 0.5;
	invT[3 * 4 + 1] = trans.y() - 0.05;
	invT[3 * 4 + 2] = trans.z() + 1.5;
	invT[3 * 4 + 3] = 1;

	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (labels[i] == leg) {
			float3 point2d = Projections::get_2d_projection(cloud->at(i), invT);
			if (point2d.y >= 0 && point2d.y < out_img.rows && point2d.x >= 0
					&& point2d.x < out_img.cols)
				out_img.at<uchar>(point2d.y, point2d.x) = 255;
		}
	}
}

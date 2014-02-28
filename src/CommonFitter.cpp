/*
 * CommonFitter.cpp
 *
 *  Created on: 24 Feb 2014
 *      Author: m04701
 */

#include "CommonFitter.h"

CommonFitter::CommonFitter(boost::shared_ptr<Skeleton> skeleton,
		boost::shared_ptr<Skeletonization3D> skeletonization3d,
		const camVecT& camera_arr) :
		camera_arr(camera_arr), enh_ik_solver(skeleton) {
	current_frame = 0;
	this->skeleton = skeleton;
	skeletonizator = skeletonization3d;
}

void CommonFitter::set_current_frame(int current_frame) {
	this->current_frame = current_frame;
}

void CommonFitter::refine_goal_position(osg::Vec3& end_position,
		const osg::Vec3& base_position, float length) {
	bone_pos_finder.refine_goal_position(end_position, base_position, length);
}

void CommonFitter::refine_start_position(osg::Vec3& start_position,
		const osg::Vec3& end_position, float length) {
	bone_pos_finder.refine_start_position(start_position, end_position, length);
}

bool CommonFitter::solve_chain(int root_bone, int end_bone,
		const osg::Vec3& position) {
	return enh_ik_solver.solve_chain(root_bone, end_bone,
			make_float3(position._v), current_frame);
}

bool CommonFitter::solve_chain_keep_next_pos(int root_bone, int end_bone,
		const osg::Vec3& position) {
	return enh_ik_solver.solve_chain_keep_next_bone_pos(root_bone, end_bone,
			make_float3(position._v), current_frame);
}

bool CommonFitter::solve_chain_keep_next_pos_ignore_res(int root_bone,
		int end_bone, const osg::Vec3& position) {
	return enh_ik_solver.solve_chain_keep_next_bone_pos(root_bone, end_bone,
			make_float3(position._v), current_frame, true);
}

bool CommonFitter::solve_chain_keep_next_pos_gradient(int root_bone,
		int end_bone, const osg::Vec3& position) {

	return enh_ik_solver.solve_chain_keep_next_pos_gradient(root_bone, end_bone,
			position, current_frame);
}

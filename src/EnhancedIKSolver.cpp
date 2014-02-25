/*
 * EnhancedIKSolver.cpp
 *
 *  Created on: 25 Feb 2014
 *      Author: m04701
 */

#include "EnhancedIKSolver.h"

EnhancedIKSolver::EnhancedIKSolver() {
}

EnhancedIKSolver::EnhancedIKSolver(boost::shared_ptr<Skeleton> skeleton) {
	init(skeleton);
}

void EnhancedIKSolver::init(boost::shared_ptr<Skeleton> skeleton) {
	this->skeleton = skeleton;
}

bool EnhancedIKSolver::solve_chain(int root_bone, int end_bone,
		const float3& position, int current_frame) {
	std::vector<int> indices;
	fill_chain(root_bone, end_bone, current_frame, indices);

	//Calculate in root_bone coordinate system where is the goal position
	osg::Matrix m;
	skeleton->get_node(root_bone)->get_node_world_matrix_origin(current_frame,
			m);
	osg::Vec3 position_osg(position.x, position.y, position.z);
	osg::Vec3 goal_position = position_osg * m;

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

bool EnhancedIKSolver::solve_chain_keep_next_bone_pos(
		unsigned int root_bone, unsigned int end_bone,
		const float3& goal_position, int current_frame) {

	//If the end bone does not have child call the simpler method
	//this one is for keep the chain after end_bone in the same state
	if (skeleton->get_node(end_bone)->get_num_children() == 0) {
		return solve_chain(root_bone, end_bone, goal_position,
				current_frame);
	}

	std::vector<int> indices;
	fill_chain(root_bone, end_bone, current_frame, indices);

	Node* next_bone = skeleton->get_node(end_bone)->children[0].get();
	osg::Vec3 next_bone_pos = next_bone->get_end_bone_global_pos(current_frame);
	osg::Quat next_bone_child_glob_prev_rot;
	if (next_bone->get_num_children() != 0) {
		next_bone_child_glob_prev_rot = next_bone->children[0]->get_global_rot(
				current_frame);
	}

	osg::Vec3 goal_pos(goal_position.x, goal_position.y, goal_position.z);

	//Calculate vector from next bone to goal position
	osg::Vec3 dir_vec = goal_pos - next_bone_pos;
	dir_vec.normalize();

	float next_bone_length = next_bone->length.length();

	//Closest point in the sphere of movement is
	//centre of the sphere (next_bone_pos) plus its radius ( next_bone_length )
	//looking at the direction of the previous goal
	goal_pos = next_bone_pos + dir_vec * next_bone_length;

	//Calculate in root_bone coordinate system where is the goal position
	osg::Matrix m;
	skeleton->get_node(root_bone)->get_node_world_matrix_origin(
			current_frame, m);

	osg::Vec3 goal_position_start_bone = goal_pos * m;

	//Solve and update the rotations
	if (ik_solver.solve_chain(make_float3(goal_position_start_bone._v))) {
		int j = 0;
		for (unsigned int i = 0; i < indices.size(); i++) {
			Node * node = skeleton->get_node(indices[i]);
			float4 new_rot;
			ik_solver.get_rotation_joint(j, new_rot);
			node->quat_arr.at(current_frame).set(new_rot.x, new_rot.y,
					new_rot.z, new_rot.w);
			j++;
		}

		//Put next bone back to where it was
		float3 next_bone_pos_f3 = make_float3(next_bone_pos._v);
		solve_chain(end_bone + 1, end_bone + 1, next_bone_pos_f3,
				current_frame);

		//Correct next bone child rotation
		//If child previous global rotation was
		//R = R3 * R2 * R1 * R0
		//Where R3 is child, R2 parent, etc
		//New state is
		//R = R3' * R2' * R1' * R0'
		//Solving for new R3' that maintains previous global rotation
		// R3' = R * inv(R2' * R1' * R0')
		if (next_bone->get_num_children() != 0) {
			next_bone->children[0]->quat_arr.at(current_frame) =
					next_bone_child_glob_prev_rot
							* next_bone->get_global_rot(current_frame).inverse();
		}
		return true;
	} else {
		return false;
	}
}

void EnhancedIKSolver::fill_chain(int root_bone,
		int end_bone, int current_frame, std::vector<int>& indices){
	ik_solver.start_chain();

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
}

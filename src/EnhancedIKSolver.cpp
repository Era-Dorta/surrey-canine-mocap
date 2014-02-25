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
		unsigned int start_bone_index, unsigned int end_bone_index,
		const float3& goal_position, int current_frame) {

}


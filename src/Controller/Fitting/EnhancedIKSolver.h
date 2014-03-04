/*
 * EnhancedIKSolver.h
 *
 *  Created on: 25 Feb 2014
 *      Author: m04701
 */

#ifndef ENHANCEDIKSOLVER_H_
#define ENHANCEDIKSOLVER_H_

#include "IKSolver.h"
#include "../../Model/Skeleton.h"

class EnhancedIKSolver {
public:
	EnhancedIKSolver();

	EnhancedIKSolver(boost::shared_ptr<Skeleton> skeleton);

	void init(boost::shared_ptr<Skeleton> skeleton);

	bool solve_chain(int root_bone, int end_bone, const float3& position,
			int current_frame);

	bool solve_chain_keep_next_bone_pos(unsigned int root_bone,
			unsigned int end_bone, const float3& goal_position,
			int current_frame, bool force_position = false);

	bool solve_chain_keep_next_pos_gradient(int root_bone, int end_bone,
			const osg::Vec3& position, int current_frame);

private:
	void fill_chain(int root_bone, int end_bone, int current_frame,
			std::vector<int>& indices);

	void undo_rotations(const std::vector<int>& indices, int current_frame);

	IKSolver ik_solver;
	std::vector<osg::Quat> previous_rotations;
	boost::shared_ptr<Skeleton> skeleton;
};

#endif /* ENHANCEDIKSOLVER_H_ */

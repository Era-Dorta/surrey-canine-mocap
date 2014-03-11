/*
 * CommonFitter.h
 *
 *  Created on: 24 Feb 2014
 *      Author: m04701
 */

#ifndef COMMONFITTER_H_
#define COMMONFITTER_H_

#include "../../Model/Skeleton.h"
#include "../Skeletonization3D.h"
#include "EnhancedIKSolver.h"
#include "BonePosFinder.h"

#include "osg/Array"
#include "opencv2/opencv.hpp"

#include "boost/shared_ptr.hpp"

class CommonFitter {
public:
	void set_current_frame(int current_frame);
protected:
	CommonFitter(SkeletonPtr skeleton, Skeletonization3DPtr skeletonization3d,
			const camVecT& camera_arr);

	bool solve_chain(int root_bone, int end_bone, const osg::Vec3& position);

	bool solve_chain_keep_next_pos(int root_bone, int end_bone,
			const osg::Vec3& position);

	bool solve_chain_keep_next_pos_ignore_res(int root_bone, int end_bone,
			const osg::Vec3& position);

	bool solve_chain_keep_next_pos_gradient(int root_bone, int end_bone,
			const osg::Vec3& position);

	int current_frame;
	Skeletonization3DPtr skeletonizator;
	SkeletonPtr skeleton;

	const camVecT& camera_arr;
	BonePosFinder bone_pos_finder;
private:
	EnhancedIKSolver enh_ik_solver;
};

#endif /* COMMONFITTER_H_ */

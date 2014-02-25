/*
 * CommonFitter.h
 *
 *  Created on: 24 Feb 2014
 *      Author: m04701
 */

#ifndef COMMONFITTER_H_
#define COMMONFITTER_H_

#include "Skeleton.h"
#include "Skeletonization3D.h"
#include "EnhancedIKSolver.h"
#include "BonePosFinder.h"

#include "osg/Array"
#include "opencv2/opencv.hpp"

#include "boost/shared_ptr.hpp"

class CommonFitter {
public:
	void set_current_frame(int current_frame);
protected:
	CommonFitter(boost::shared_ptr<Skeleton> skeleton,
			boost::shared_ptr<Skeletonization3D> skeletonization3d,
			const camVecT& camera_arr);

	void refine_goal_position(osg::Vec3& end_position,
			const osg::Vec3& base_position, float length);

	void refine_start_position(osg::Vec3& start_position,
			const osg::Vec3& end_position, float length);

	bool solve_chain(int root_bone, int end_bone, const osg::Vec3& position);

	const std::vector<Skeleton::Skel_Leg>& getLabels() const;

	float move_joint_max_dist;
	float error_threshold;
	int current_frame;
	boost::shared_ptr<Skeletonization3D> skeletonizator;
	boost::shared_ptr<Skeleton> skeleton;

	EnhancedIKSolver enh_ik_solver;
	const camVecT& camera_arr;
	BonePosFinder bone_pos_finder;
};

#endif /* COMMONFITTER_H_ */

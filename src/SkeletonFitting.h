/*
 * SkeletonFitting.h
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#ifndef SKELETONFITTING_H_
#define SKELETONFITTING_H_

#include "Skeleton.h"
#include "Skeletonization3D.h"
#include "IKSolver.h"
#include "CloudClusterer.h"
#include "BonePosFinder.h"

#include "osg/Array"
#include "opencv2/opencv.hpp"

#include "boost/shared_ptr.hpp"

class SkeletonFitting {
public:
	SkeletonFitting(boost::shared_ptr<Skeleton> skeleton_,
			boost::shared_ptr<Skeletonization3D> skeletonization3d,
			const camVecT& camera_arr);
	virtual ~SkeletonFitting();

	void calculate_for_frame(int frame_num);

	void fit_skeleton_to_cloud();

	const std::vector<Skeleton::Skel_Leg>& getLabels() const;

	osg::Vec3 get_paw(Skeleton::Skel_Leg leg);

	bool solve_chain(int root_bone, int end_bone, const osg::Vec3& position);

	void calculate_bone_world_matrix_origin(osg::Matrix& matrix,
			const Node* const node);
private:
	bool fit_root_position();

	bool fit_head_and_back();

	bool fit_leg_position_complete(Skeleton::Skel_Leg leg);

	bool fit_leg_position(Skeleton::Skel_Leg leg);

	bool fit_leg_position_simple(Skeleton::Skel_Leg leg);

	bool fit_leg_position_simple(Skeleton::Skel_Leg leg, int paw_index,
			std::vector<int>& leg_points_index);

	bool fit_leg_position_mid_pos_in_top_leg(Skeleton::Skel_Leg leg,
			int& paw_index);

	bool fit_leg_position_mid_pos_in_top_leg(Skeleton::Skel_Leg leg,
			int& paw_index, std::vector<int>& leg_points_index);

	bool fit_leg_position_half_way(Skeleton::Skel_Leg leg, int paw_index);

	bool fix_leg_second_lower_joint(Skeleton::Skel_Leg leg,
			const std::vector<int>& leg_points_index);

	bool fit_leg_pos_impl(Skeleton::Skel_Leg leg, const osg::Vec3& paw_position,
			const osg::Vec3& middle_position);

	void refine_goal_position(osg::Vec3& end_position,
			const osg::Vec3& base_position, float length);

	float calculate_sum_distance2_to_cloud(int index,
			const std::vector<int>& leg_points_index);

	void reduce_points_with_height(float max_y, float min_y,
			const std::vector<int>& leg_points_index,
			std::vector<int>& new_leg_points_index);

	//Needed to use std::sort with comp_y
	struct sortstruct {
		// sortstruct needs to know its containing object
		SkeletonFitting* m;
		bool (*comp_funct)(const osg::Vec3&, const osg::Vec3&);
		sortstruct(SkeletonFitting* p,
				bool (*comp_funct_)(const osg::Vec3&, const osg::Vec3&)) :
				m(p) {
			m = p;
			comp_funct = comp_funct_;
		}
		;

		bool operator()(int i, int j) {
			return (!comp_funct(m->cloud->at(i), m->cloud->at(j)));
		}

		bool operator()(const osg::Vec3& i, const osg::Vec3& j) {
			return (!comp_funct(i, j));
		}
	};

	float move_joint_max_dist;
	float error_threshold;
	float mean_z_front_all_frames;
	float mean_z_back_all_frames;
	std::vector<Skeleton::Skel_Leg> labels;
	int current_frame;
	int n_frames;
	osg::ref_ptr<osg::Vec3Array> cloud;
	boost::shared_ptr<Skeletonization3D> skeletonizator;
	boost::shared_ptr<Skeleton> skeleton;
	bool first_call;

	IKSolver ik_solver;
	const camVecT& camera_arr;
	CloudClusterer cloud_clusterer;
	BonePosFinder bone_pos_finder;
};

#endif /* SKELETONFITTING_H_ */

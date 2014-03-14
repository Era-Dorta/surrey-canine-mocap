/*
 * LegFitter.h
 *
 *  Created on: 24 Feb 2014
 *      Author: m04701
 */

#ifndef LEGFITTER_H_
#define LEGFITTER_H_

#include "CommonFitter.h"
#include "CompMethods.h"

class LegFitter: public CommonFitter {
public:
	LegFitter(SkeletonPtr skeleton, Skeletonization3DPtr skeletonization3d,
			const camVecT& camera_arr);

	bool fit_leg_position_complete(Skeleton::Skel_Leg leg,
			const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels);
private:
	bool fit_leg_position_initialise(Skeleton::Skel_Leg leg, int& paw_index,
			std::vector<int>& leg_points_index,
			const std::vector<Skeleton::Skel_Leg>& labels);

	bool fit_leg_position_go_up_y(Skeleton::Skel_Leg leg, int paw_index,
			const std::vector<int>& leg_points_index);

	bool fit_leg_position_mid_pos_in_top_leg(Skeleton::Skel_Leg leg,
			int paw_index, const std::vector<int>& leg_points_index,
			const std::vector<Skeleton::Skel_Leg>& labels);

	bool fit_leg_position_half_way(Skeleton::Skel_Leg leg, int paw_index);

	bool fix_leg_second_lower_joint(Skeleton::Skel_Leg leg,
			const std::vector<int>& leg_points_index);

	bool fix_leg_second_lower_joint(Skeleton::Skel_Leg leg,
			const osg::Vec3& goal_pos);

	bool fit_leg_pos_impl(Skeleton::Skel_Leg leg, const osg::Vec3& paw_position,
			const osg::Vec3& middle_position);

	float calculate_sum_distance2_to_cloud(const osg::Vec3& bone_end_pos,
			const std::vector<int>& leg_points_index);

	void reduce_points_with_height(float max_y, float min_y,
			const std::vector<int>& leg_points_index,
			std::vector<int>& new_leg_points_index);

	//Pos0 is paw position, and pos1 and pos2 of the respective parent bones
	bool solve_leg_3_pos(Skeleton::Skel_Leg leg, const osg::Vec3& pos0,
			const osg::Vec3& pos1, const osg::Vec3& pos2);

	int solve_leg_2_pos_approx(Skeleton::Skel_Leg leg, const osg::Vec3& pos1,
			const osg::Vec3& pos2);

	bool solve_leg_upper_pos_approx(Skeleton::Skel_Leg leg,
			const osg::Vec3& pos);

	//Needed to use std::sort with comp_y
	struct sortstruct {
		// sortstruct needs to know its containing object
		LegFitter* m;
		bool (*comp_funct)(const osg::Vec3&, const osg::Vec3&);
		sortstruct(LegFitter* p,
				bool (*comp_funct_)(const osg::Vec3&, const osg::Vec3&)) :
				m(p) {
			m = p;
			comp_funct = comp_funct_;
		}
		;

		bool operator()(int i, int j) {
			return (!comp_funct(m->cloud->get_osg(i), m->cloud->get_osg(j)));
		}

		bool operator()(const osg::Vec3& i, const osg::Vec3& j) {
			return (!comp_funct(i, j));
		}
	};

	PointCloudPtr cloud;
};

#endif /* LEGFITTER_H_ */

/*
 * SkeletonFitting.h
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#ifndef SKELETONFITTING_H_
#define SKELETONFITTING_H_

//#include "Skeleton.h"
#include "osg/Array"
#include <vector>
#include <algorithm>
#include "opencv2/opencv.hpp"

enum Skel_Leg {
	Front_Left, Front_Right, Back_Left, Back_Right, Not_Use
};

enum Axis {
	X, Y, Z
};

class SkeletonFitting {
	public:
		SkeletonFitting();
		virtual ~SkeletonFitting();
		//void fit_skeleton_into_cloud(Skeleton& skeleton,
		//		osg::ref_ptr<osg::Vec3Array> cloud);
		//void fit_skeleton_with_prev_nex_frame(Skeleton& skeleton, int frame);

		//Median gives better results that mean, but it is not as fast
		void divide_four_sections(osg::ref_ptr<osg::Vec3Array> cloud,
				std::vector<Skel_Leg>& result, bool use_median = true);
	private:
		//Divide all the skeleton points in left and right using z distance
		//Divide right along x
		//from that group take the lowest one
		void find_front_right_paw();

		float get_median(osg::ref_ptr<osg::Vec3Array> points,
				std::vector<Skel_Leg>& labels, Skel_Leg use_label, Axis axis);

		float get_mean(osg::ref_ptr<osg::Vec3Array> points,
				std::vector<Skel_Leg>& labels, Skel_Leg use_label, Axis axis);

		float move_joint_max_dist;
};

#endif /* SKELETONFITTING_H_ */

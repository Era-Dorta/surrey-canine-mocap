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
#include "IKAN/srs.h"

#include "osg/Array"
#include "opencv2/opencv.hpp"

#include <vector>
#include <algorithm>

#include "boost/shared_ptr.hpp"

enum Skel_Leg {
	Front_Left, Front_Right, Back_Left, Back_Right, Not_Limbs
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
		void init(boost::shared_ptr<Skeleton> skeleton_,
				boost::shared_ptr<Skeletonization3D> skeletonization3d);

		void calculate_for_frame(int frame_num);

		void fit_root_position();

		void fit_leg_position(Skel_Leg leg);

		const std::vector<Skel_Leg>& getLabels() const;

		osg::Vec3 get_paw(Skel_Leg leg);
	private:
		int find_head();

		int find_paw(Skel_Leg leg);

		//From a cloud of points, fill result vector with a label for each point
		//Median gives better results that mean, but it is not as fast
		void divide_four_sections(bool use_median = true);

		bool solve_2_bones(int bone0, int bone1, const osg::Vec3& position);

		float get_median(osg::ref_ptr<osg::Vec3Array> points,
				Skel_Leg use_label, Axis axis);

		float get_mean(osg::ref_ptr<osg::Vec3Array> points, Skel_Leg use_label,
				Axis axis);

		bool are_equal(const osg::Vec3& v0, const osg::Vec3& v1);

		void osg_to_matrix(Matrix &dest, const osg::Matrix &orig);

		void matrix_to_osg(osg::Matrix& dest, const Matrix& orig);

		float move_joint_max_dist;
		float error_threshold;
		std::vector<Skel_Leg> labels;
		int current_frame;
		osg::ref_ptr<osg::Vec3Array> cloud;
		boost::shared_ptr<Skeletonization3D> skeletonizator;
		boost::shared_ptr<Skeleton> skeleton;
};

#endif /* SKELETONFITTING_H_ */

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
#include "PixelSearch.h"
#include "IKAN/srs.h"

#include "osg/Array"
#include "opencv2/opencv.hpp"

#include <vector>
#include <algorithm>

#include "boost/shared_ptr.hpp"

//Values correspond to paw Node indices in skeleton
enum Skel_Leg {
	Front_Left = 5,
	Front_Right = 9,
	Back_Left = 14,
	Back_Right = 18,
	Not_Limbs = 0
};

enum Axis {
	X, Y, Z
};

bool comp_y(const osg::Vec3& i, const osg::Vec3& j);

class SkeletonFitting {
	public:
		SkeletonFitting(boost::shared_ptr<Skeleton> skeleton_,
				boost::shared_ptr<Skeletonization3D> skeletonization3d);
		virtual ~SkeletonFitting();
		//void fit_skeleton_into_cloud(Skeleton& skeleton,
		//		osg::ref_ptr<osg::Vec3Array> cloud);
		//void fit_skeleton_with_prev_nex_frame(Skeleton& skeleton, int frame);

		void calculate_for_frame(int frame_num);

		void fit_root_position();

		void fit_leg_position(Skel_Leg leg);

		void fit_vertebral_front();

		const std::vector<Skel_Leg>& getLabels() const;

		osg::Vec3 get_paw(Skel_Leg leg);

		bool solve_2_bones(int bone0, const osg::Vec3& position0, int bone1,
				const osg::Vec3& position1);

		bool solve_2_bones(int bone0, int bone1, const osg::Vec3& position,
				float swivel_angle = 0.0);

		float get_swivel_angle(int bone0, int bone1);
	private:
		int find_head();

		int find_paw(Skel_Leg leg, std::vector<int>& leg_points_index);

		//From a cloud of points, fill result vector with a label for each point
		//Median gives better results that mean, but it is not as fast
		void divide_four_sections();

		float get_mean(osg::ref_ptr<osg::Vec3Array> points, Skel_Leg use_label,
				Axis axis);

		bool are_equal(const osg::Vec3& v0, const osg::Vec3& v1);

		void osg_to_matrix(Matrix &dest, const osg::Matrix &orig);

		void matrix_to_osg(osg::Matrix& dest, const Matrix& orig);

		bool check_bone_index(int bone0, int bone1);

		bool unstuck_go_down(const cv::Mat& img, int i_row, int i_col,
				int &res_row, int &res_col);

		bool solve_2_bones_impl(int bone0, const osg::Vec3& position0,
				int bone1, const osg::Vec3& position1, float swivel_angle,
				bool use_swivel);

		void calculate_bone_world_matrix_origin(osg::Matrix& matrix,
				const Node* const node);

		void refine_goal_position(osg::Vec3& end_position,
				const osg::Vec3& base_position, float length);

		//Needed to use std::sort with comp_y
		struct sortstruct {
				// sortstruct needs to know its containing object
				SkeletonFitting* m;
				sortstruct(SkeletonFitting* p) :
							m(p) {
				}
				;

				bool operator()(int i, int j) {
					return (!comp_y(m->cloud->at(i), m->cloud->at(j)));
				}
		};

		float move_joint_max_dist;
		float error_threshold;
		float body_height_extra_threshold;
		std::vector<Skel_Leg> labels;
		int current_frame;
		osg::ref_ptr<osg::Vec3Array> cloud;
		boost::shared_ptr<Skeletonization3D> skeletonizator;
		boost::shared_ptr<Skeleton> skeleton;
};

#endif /* SKELETONFITTING_H_ */

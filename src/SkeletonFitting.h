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
#include "Projections.h"
#include "KNNSearch.h"
#include "IKSolver.h"

#include "osg/Array"
#include "opencv2/opencv.hpp"

#include <vector>
#include <algorithm>

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

		bool solve_chain(int root_bone, int end_bone,
				const osg::Vec3& position);

		void calculate_bone_world_matrix_origin(osg::Matrix& matrix,
				const Node* const node);
	private:
		bool fit_root_position();

		bool fit_leg_position(Skeleton::Skel_Leg leg);

		bool fit_leg_position_simple(Skeleton::Skel_Leg leg);

		bool fit_leg_position_mid_pos_in_top_leg(Skeleton::Skel_Leg leg);

		bool fit_head_and_back();

		int find_head();

		int find_paw(Skeleton::Skel_Leg leg, std::vector<int>& leg_points_index);

		int find_leg_upper_end(Skeleton::Skel_Leg leg,
				std::vector<int>& leg_points_index);

		bool find_first_bone_end_pos(const osg::Vec3& root_pos,
				osg::Vec3& head_pos);

		bool find_second_bone_end_pos(const osg::Vec3& head_pos,
				osg::Vec3& shoulder_pos);

		bool find_vertebral_end_pos(const osg::Vec3& shoulder_pos,
				osg::Vec3& vertebral_end_pos);

		bool fit_leg_pos_impl(Skeleton::Skel_Leg leg, const osg::Vec3& paw_position,
				const osg::Vec3& middle_position);

		//From a cloud of points, fill result vector with a label for each point
		//Median gives better results that mean, but it is not as fast
		void divide_four_sections(bool use_simple_division = true);

		void refine_four_sections_division();

		float get_mean(osg::ref_ptr<osg::Vec3Array> points, Skeleton::Skel_Leg use_label,
				Skeleton::Axis axes);

		float get_division_val(osg::ref_ptr<osg::Vec3Array> points,
				Skeleton::Skel_Leg use_label, Skeleton::Axis axes);

		bool are_equal(const osg::Vec3& v0, const osg::Vec3& v1);

		bool check_bone_index(int bone0, int bone1);

		bool unstuck_go_down(const cv::Mat& img, int i_row, int i_col,
				int &res_row, int &res_col);

		void refine_goal_position(osg::Vec3& end_position,
				const osg::Vec3& base_position, float length);

		void recalculate_front_back_division_side_view();

		void recalculate_right_left_division_time_coherence();

		void recalculate_right_left_division_mass_center();

		void recalculate_right_left_division_front_view();

		void recalculate_right_left_division_back_view();

		void recalculate_right_left_knn(unsigned int num_nn = 50,
				float max_distance_threshold = 0.01, unsigned int max_ite = 10);

		bool knn_division_done(unsigned int num_nn,
				const std::vector<std::vector<int> >& indices,
				const std::vector<std::vector<float> >& dists,
				float min_distance_treshold = 0.0005);

		bool reclassify_left_right_leg_points(float mean_z_front,
				float mean_z_back);

		void get_y_z_front_projection(Skeleton::Skel_Leg leg, cv::Mat& out_img,
				const osg::Vec3& trans = osg::Vec3());

		void get_x_y_side_projection(Skeleton::Skel_Leg leg, cv::Mat& out_img,
				const osg::Vec3& trans = osg::Vec3());

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
		float body_height_extra_threshold;
		std::vector<float> mean_z_front_arr;
		std::vector<float> mean_z_back_arr;
		std::vector<float> mean_x_arr;
		float mean_z_front_all_frames;
		float mean_z_back_all_frames;
		std::vector<Skeleton::Skel_Leg> labels;
		int current_frame;
		int n_frames;
		osg::ref_ptr<osg::Vec3Array> cloud;
		boost::shared_ptr<Skeletonization3D> skeletonizator;
		boost::shared_ptr<Skeleton> skeleton;
		KNNSearch knn_searcher;
		IKSolver ik_solver;
		const camVecT& camera_arr;
};

#endif /* SKELETONFITTING_H_ */

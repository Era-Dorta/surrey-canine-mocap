/*
 * CloudClusterer.h
 *
 *  Created on: 12 Feb 2014
 *      Author: m04701
 */

#ifndef CLOUDCLUSTERER_H_
#define CLOUDCLUSTERER_H_

#include "KNNSearch.h"
#include "../../Model/Skeleton.h"
#include "../Projections.h"
#include "../PixelSearch.h"
#include "BonePosFinder.h"
#include "CompMethods.h"

#include "../../Misc/CudaVec.h"

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

class CloudClusterer {
public:
	CloudClusterer();

	CloudClusterer(int n_frames, int img_rows, int img_cols);

	void init(int n_frames, int img_rows, int img_cols);

	//From a cloud of points, fill result vector with a label for each point
	//Median gives better results that mean, but it is not as fast
	void divide_four_sections(const PointCloudPtr& point_cloud,
			std::vector<Skeleton::Skel_Leg>& labels, int frame_num,
			bool use_simple_division = true);

	static bool comp_x(const osg::Vec3& i, const osg::Vec3& j);

	static bool comp_y(const osg::Vec3& i, const osg::Vec3& j);

	static bool comp_z(const osg::Vec3& i, const osg::Vec3& j);
private:
	void divide_four_sections_simple(std::vector<Skeleton::Skel_Leg>& labels);

	void divide_four_sections_knn(std::vector<Skeleton::Skel_Leg>& labels,
			int num_invalid);

	void refine_four_sections_division(const PointCloudPtr& point_cloud,
			std::vector<Skeleton::Skel_Leg>& labels, int frame_num,
			int head_index);

	void recalculate_front_back_division_side_view(
			std::vector<Skeleton::Skel_Leg>& labels, int head_index);

	void recalculate_right_left_division_time_coherence(
			std::vector<Skeleton::Skel_Leg>& labels);

	void recalculate_right_left_division_mass_center(
			std::vector<Skeleton::Skel_Leg>& labels);

	void recalculate_right_left_division_front_view(
			std::vector<Skeleton::Skel_Leg>& labels, int head_index);

	void recalculate_right_left_division_back_view(
			std::vector<Skeleton::Skel_Leg>& labels, int head_index);

	void recalculate_right_left_knn(std::vector<Skeleton::Skel_Leg>& labels,
			unsigned int num_nn = 50, float max_distance_threshold = 0.01,
			unsigned int max_ite = 10);

	bool knn_division_done(std::vector<Skeleton::Skel_Leg>& labels,
			unsigned int num_nn, const std::vector<std::vector<int> >& indices,
			const std::vector<std::vector<float> >& dists,
			float min_distance_treshold = 0.0005);

	bool reclassify_left_right_leg_points(
			std::vector<Skeleton::Skel_Leg>& labels, float mean_z_front,
			float mean_z_back);

	float get_mean(const PointCloudPtr& points,
			std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg use_label, Skeleton::Axis axis);

	float get_division_val(const PointCloudPtr& points,
			std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg use_label, Skeleton::Axis axis);

	PointCloudPtr cloud;
	int current_frame;
	int n_frames;
	int img_rows;
	int img_cols;
	std::vector<float> mean_z_front_arr;
	std::vector<float> mean_z_back_arr;
	std::vector<float> mean_x_arr;
	float body_height_extra_threshold;
	KNNSearch knn_searcher;
	BonePosFinder bone_pos_finder;
};

#endif /* CLOUDCLUSTERER_H_ */

/*
 * BonePosFinder.h
 *
 *  Created on: 12 Feb 2014
 *      Author: m04701
 */

#ifndef BONEPOSFINDER_H_
#define BONEPOSFINDER_H_

#include "../PixelSearch.h"
#include "../Projections.h"
#include "../../Model/Skeleton.h"
#include "../../Model/PointCloud.h"

//#include <pcl/io/pcd_io.h>

#include <pcl/common/common.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <opencv2/opencv.hpp>

class BonePosFinder {
public:
	BonePosFinder();

	int find_head(const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels);

	int find_paw(const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg leg, std::vector<int>& leg_points_index);

	//Only call fast methods with an initialised and ordered leg_point_index
	int find_paw_fast(const std::vector<int>& leg_points_index);

	int find_leg_upper_end(const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg leg, std::vector<int>& leg_points_index);

	//Only call fast methods with an initialised and ordered leg_point_index
	int find_leg_upper_end_fast(const std::vector<int>& leg_points_index);

	bool find_first_bone_end_pos(const cv::Mat& cam2_bin_img, float bone_length,
			constCamVecIte& cam, int current_frame, const osg::Vec3& root_pos,
			osg::Vec3& head_pos);

	bool find_second_bone_end_pos(const cv::Mat& cam1_bin_img,
			float bone_length, constCamVecIte& cam, int current_frame,
			const osg::Vec3& head_pos, osg::Vec3& shoulder_pos);

	bool find_vertebral_end_pos(const cv::Mat& cam1_bin_img, float bone_length,
			constCamVecIte& cam, int current_frame,
			const osg::Vec3& shoulder_pos, osg::Vec3& vertebral_end_pos);

	int find_leg_lower_3_joints_simple(const PointCloudPtr& cloud,
			const std::vector<int>& leg_points_index,
			const float bone_lengths[2], osg::Vec3 bone_positions[3]);

	int find_leg_lower_3_joints_line_fitting(const PointCloudPtr& cloud,
			const std::vector<int>& leg_points_index,
			const float bone_lengths[2], const osg::Vec3 prev_bone_positions[3],
			osg::Vec3 new_bone_positions[3]);

	void refine_goal_position(osg::Vec3& end_position,
			const osg::Vec3& base_position, float length);

	void refine_start_position(osg::Vec3& start_position,
			const osg::Vec3& end_position, float length);

private:
	bool unstuck_go_down(const cv::Mat& img, int i_row, int i_col, int &res_row,
			int &res_col);

	void get_y_z_front_projection(const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg leg, cv::Mat& out_img, const osg::Vec3& trans =
					osg::Vec3());

	void get_x_y_side_projection(const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg leg, cv::Mat& out_img, const osg::Vec3& trans =
					osg::Vec3());

	bool fit_line_to_cloud(const std::vector<cv::Point3f>& bone_points,
			osg::Vec3& line_vec, osg::Vec3& line_point);

	//Returns in res_point the closest point in line from from_point
	void closest_point_in_line_from_point(const osg::Vec3& from_point,
			const osg::Vec3& line_vec, const osg::Vec3& line_point,
			osg::Vec3& res_point);

	//Returns the point with highest Y (most up in osg) value in line
	//at distance from from_point
	bool point_in_line_given_distance_most_up(const osg::Vec3& from_point,
			float distance, const osg::Vec3& line_vec,
			const osg::Vec3& line_point, osg::Vec3& res_point);

	//Returns the point with lowest x (most to the left in osg) value in line
	//at distance from from_point
	bool point_in_line_given_distance_most_left(const osg::Vec3& from_point,
			float distance, const osg::Vec3& line_vec,
			const osg::Vec3& line_point, osg::Vec3& res_point);

	bool sphere_line_intersection(const osg::Vec3& sphe_centre, float radius,
			const osg::Vec3& line_vec, const osg::Vec3& line_point,
			osg::Vec3& res_point0, osg::Vec3& res_point1);
};

#endif /* BONEPOSFINDER_H_ */

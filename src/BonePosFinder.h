/*
 * BonePosFinder.h
 *
 *  Created on: 12 Feb 2014
 *      Author: m04701
 */

#ifndef BONEPOSFINDER_H_
#define BONEPOSFINDER_H_

#include "PixelSearch.h"
#include "Projections.h"
#include "Skeleton.h"

class BonePosFinder {
public:
	BonePosFinder();

	int find_head(const osg::ref_ptr<osg::Vec3Array>& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels);

	int find_paw(const osg::ref_ptr<osg::Vec3Array>& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg leg, std::vector<int>& leg_points_index);

	int find_leg_upper_end(const osg::ref_ptr<osg::Vec3Array>& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels,
			Skeleton::Skel_Leg leg, std::vector<int>& leg_points_index);

	bool find_first_bone_end_pos(const cv::Mat& cam2_bin_img, float bone_length,
			constCamVecIte& cam, int current_frame, const osg::Vec3& root_pos,
			osg::Vec3& head_pos);

	bool find_second_bone_end_pos(const cv::Mat& cam1_bin_img,
			float bone_length, constCamVecIte& cam, int current_frame,
			const osg::Vec3& head_pos, osg::Vec3& shoulder_pos);

	bool find_vertebral_end_pos(const cv::Mat& cam1_bin_img, float bone_length,
			constCamVecIte& cam, int current_frame,
			const osg::Vec3& shoulder_pos, osg::Vec3& vertebral_end_pos);

	bool unstuck_go_down(const cv::Mat& img, int i_row, int i_col, int &res_row,
			int &res_col);
};

#endif /* BONEPOSFINDER_H_ */

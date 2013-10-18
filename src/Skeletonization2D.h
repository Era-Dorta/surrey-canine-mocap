#ifndef SKELETONIZATION2D_H
#define SKELETONIZATION2D_H

#include "RGBDFrame.h"

#include "opencv2/opencv.hpp"

#include <iostream>
using std::cout;
using std::endl;

class Skeletonization2D
{
	public:
		Skeletonization2D(std::map<int, RGBD_Frame>* camera_frames);
		virtual ~Skeletonization2D();
	protected:
	private:
		void generate_skeletonization();
		osg::ref_ptr<osg::Vec3Array> points_from_image(const cv::Mat& seg_img);
		cv::Mat dist_transform_skeletonization(const cv::Mat& seg_img);
		cv::Mat connectivity_preserving_thinning(cv::Mat& img_in);
		cv::Mat remove_isolated_short_segments(cv::Mat& img_in, int thresh_length);

		std::vector< cv::Mat > skeletonized_frames;
		std::map<int, RGBD_Frame>* frames;
};

#endif // SKELETONIZATION2D_H

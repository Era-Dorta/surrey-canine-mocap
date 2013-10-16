#ifndef SKELETONIZATION_H
#define SKELETONIZATION_H

#include "RGBDFrame.h"

#include "opencv2/opencv.hpp"
#include <iostream>

class Skeletonization
{
	public:
		Skeletonization(std::map<int, RGBD_Frame>* camera_frames);
		virtual ~Skeletonization();
		void generate_skeletonization();
	protected:
	private:
		cv::Mat dist_transform_skeletonization(cv::Mat& seg_img);
		cv::Mat connectivity_preserving_thinning(cv::Mat img_in);
		cv::Mat remove_isolated_short_segments(cv::Mat img_in, int thresh_length);

		std::vector<cv::Mat> skeletonized_imgs;
		std::map<int, RGBD_Frame>* frames;
};

#endif // SKELETONIZATION_H

#ifndef SKELETONIZATION_H
#define SKELETONIZATION_H

#include "opencv2/opencv.hpp"
#include <iostream>

class Skeletonization
{
	public:
		Skeletonization();
		virtual ~Skeletonization();
		void dist_transform_skeletonization(cv::Mat& seg_img);
	protected:
	private:
		cv::Mat connectivity_preserving_thinning(cv::Mat img_in);
		cv::Mat remove_isolated_short_segments(cv::Mat img_in, int thresh_length);
};

#endif // SKELETONIZATION_H

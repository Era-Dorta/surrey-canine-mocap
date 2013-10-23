#ifndef SKELETONIZATION2D_H
#define SKELETONIZATION2D_H

#include "RGBDCamera.h"

#include "opencv2/opencv.hpp"

#include <iostream>
using std::cout;
using std::endl;

class Skeletonization2D
{
	public:
		Skeletonization2D(const boost::shared_ptr<RGBD_Camera> camera_);
		virtual ~Skeletonization2D();
		const cv::Mat* const get_frame(const int frame_num ) const;
	protected:
	private:
		void generate_skeletonization();
		osg::ref_ptr<osg::Vec3Array> points_from_image(const cv::Mat& seg_img);
		cv::Mat dist_transform_skeletonization(const cv::Mat* seg_img);
		cv::Mat connectivity_preserving_thinning(cv::Mat& img_in);
		cv::Mat remove_isolated_short_segments(cv::Mat& img_in,unsigned int thresh_length);

		std::vector< cv::Mat > skeletonized_frames;

		boost::shared_ptr<RGBD_Camera> camera;
};

#endif // SKELETONIZATION2D_H

#ifndef SKELETONIZATION_H
#define SKELETONIZATION_H

#include "RGBDFrame.h"

#include "opencv2/opencv.hpp"

#include <iostream>
using std::cout;
using std::endl;

class Skeletonization
{
	public:
		Skeletonization(std::map<int, RGBD_Frame>* camera_frames);
		virtual ~Skeletonization();
		osg::ref_ptr<osg::Vec3Array> get_points_for_frame( int frame );
	protected:
	private:
		void generate_skeletonization();
		osg::ref_ptr<osg::Vec3Array> points_from_image(const cv::Mat& seg_img);
		cv::Mat dist_transform_skeletonization(const cv::Mat& seg_img);
		cv::Mat connectivity_preserving_thinning(cv::Mat& img_in);
		cv::Mat remove_isolated_short_segments(cv::Mat& img_in, int thresh_length);

		std::vector< osg::ref_ptr<osg::Vec3Array> > skeletonized_points;
		std::map<int, RGBD_Frame>* frames;
};

#endif // SKELETONIZATION_H

#ifndef SKELETONIZATION2D_H
#define SKELETONIZATION2D_H

#include "../View/RGBDCamera.h"
#include "PixelSearch.h"

#include <opencv2/opencv.hpp>

#include <iostream>
using std::cout;
using std::endl;

class Skeletonization2D {
public:
	Skeletonization2D(const RGBD_CameraPtr camera_, int max_clusters_ = 6);
	virtual ~Skeletonization2D();
	const cv::Mat& get_frame(int frame_num) const;
	const cv::Mat& get_bin_frame(int frame_num) const;
	void generate_skeletonization();
protected:
private:
	osg::ref_ptr<osg::Vec3Array> points_from_image(const cv::Mat& seg_img);

	void connectivity_preserving_thinning(const cv::Mat& img_in,
			cv::Mat& result);

	void remove_isolated_short_segments(const cv::Mat& img_in, cv::Mat& res,
			unsigned int thresh_length);

	void dist_transform_skeletonization(const cv::Mat* seg_img, cv::Mat& res);
	//TODO This takes the most top white pixel and follows its line to delete
	//the arm, it is quite naive. Should do some segmentation or other
	//complicated algorithm
	void delete_arm(cv::Mat& img_in);

	std::vector<cv::Mat> skeletonized_frames;
	std::vector<cv::Mat> bin_frames;

	RGBD_CameraPtr camera;

	int max_clusters;
};

typedef boost::shared_ptr<Skeletonization2D> Skeletonization2DPtr;

#endif // SKELETONIZATION2D_H

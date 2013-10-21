#ifndef SKELETONIZATION3D_H
#define SKELETONIZATION3D_H

#include "Skeletonization2D.h"
#include "RGBDCamera.h"

#include <vector>

#include "boost/shared_ptr.hpp"

class Skeletonization3D
{
	public:
		Skeletonization3D();
		virtual ~Skeletonization3D();
		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_);
	protected:
	private:
		void merge_2D_skeletons();
		osg::ref_ptr<osg::Vec3Array> merge_2D_skeletons_impl(std::vector< cv::Mat* >& skeletonized_frames, int frame_num);
		bool get_white_pixel( cv::Mat* img, int &res_row, int &res_col );
		//Vector of Skeletonization class, there is one instance
		//for each camera
		std::vector < boost::shared_ptr<Skeletonization2D> > skel_arr;

		//Each Vec3Array is a cloud of points that represent a skeleton
		//in a given frame
		std::vector < osg::ref_ptr<osg::Vec3Array> > skeleton_frames;

		//This pointer is only valid as long as the MultiCameraViewer camera
		//holds the memory Change this and the camera to boost pointers
		std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr;

		std::vector < cv::Mat > visited_pixels;

		int n_cameras;
		int n_frames;
};

#endif // SKELETONIZATION3D_H

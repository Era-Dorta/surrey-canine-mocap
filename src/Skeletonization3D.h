#ifndef SKELETONIZATION3D_H
#define SKELETONIZATION3D_H

#include "Skeletonization2D.h"
#include "RGBDCamera.h"

#include <vector>

#include "boost/shared_ptr.hpp"

class Skeletonization3D
{
	public:
		Skeletonization3D( float merge_treshold_ = 5 );

		virtual ~Skeletonization3D();

		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> > camera_arr_);

		//Return an array of points, given a camera and a frame number
		osg::ref_ptr<osg::Vec3Array> get_simple_3d_projection( int cam_num, int frame_num ) const;

		//Get a 2D skeleton frame
		const cv::Mat* const get_2D_frame(int cam_num, int frame_num ) const;
	protected:
	private:
		//Merges several 2D images to a 3D complete image of a skeleton
		void merge_2D_skeletons();

		//Internal method that does the hard work
		osg::ref_ptr<osg::Vec3Array> merge_2D_skeletons_impl(std::vector<const cv::Mat* >& skeletonized_frames, int frame_num);

		//Auxiliary method that finds a withe pixel in a given image and returns
		//where in res_row and res_col
		bool get_white_pixel( cv::Mat* img, int &res_row, int &res_col );

		//Vector of Skeletonization class, there is one instance
		//for each camera
		std::vector < boost::shared_ptr<Skeletonization2D> > skel_arr;

		//Each Vec3Array is a cloud of points that represent a skeleton
		//in a given frame
		std::vector < osg::ref_ptr<osg::Vec3Array> > skeleton_frames;

		//There will not be many cameras, so a copy of the camera vector is not
		//that painful to do and lest assume the cameras are not going to change
		//in run time
		std::vector < boost::shared_ptr<RGBD_Camera> > camera_arr;

		std::vector < cv::Mat > visited_pixels;

		int n_cameras;
		int n_frames;
		float merge_treshold;
};

#endif // SKELETONIZATION3D_H

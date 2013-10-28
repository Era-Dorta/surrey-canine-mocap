#ifndef SKELETONIZATION3D_H
#define SKELETONIZATION3D_H

#include "Skeletonization2D.h"
#include "RGBDCamera.h"

#include <vector>

#include "boost/shared_ptr.hpp"

class Skeletonization3D
{
	public:
		Skeletonization3D( float merge_treshold_ = 0.2 );

		virtual ~Skeletonization3D();

		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> > camera_arr_);

		//Return an array of points, given a camera and a frame number
		//Important: This coordinates are relative to the camera
		osg::ref_ptr<osg::Vec3Array> get_simple_3d_projection( int cam_num, int frame_num) const;

		//Return an array of points, after merging all the camera views
		//Important: This coordinates are global
		osg::ref_ptr<osg::Vec3Array> get_merged_3d_projection( int frame_num) const;

		//Get a 2D skeleton frame
		const cv::Mat* const get_2D_frame(int cam_num, int frame_num ) const;
	protected:
	private:
		//Merges several 2D images to a 3D complete image of a skeleton
		void merge_2D_skeletons();

		//Internal method that does the hard work
		osg::ref_ptr<osg::Vec3Array> merge_2D_skeletons_impl(
				std::vector<const cv::Mat* >& skeletonized_frames, int frame_num);

		//Auxiliary method that finds a white pixel in a given image and returns
		//where in res_row and res_col
		bool get_white_pixel( cv::Mat* img, int &res_row, int &res_col, int i_row = 0, int i_col = 0 );

		//Auxiliary method that finds the white pixel situated most at the
		//bottom left of the image.
		bool get_bottom_white_pixel( cv::Mat* img, int &res_row, int &res_col );

		void get_global_coord_3d_projection(int cam_num, int frame_num, std::map<osg::Vec2, osg::Vec3>& projection3d) const;

		//merge_2D_skeletons_impl can call this two methods to actually do the
		//merging.

		//This method finds a white pixel starting from 0,0 and then calculates
		//distances to all white pixels in the other camera views, then takes the
		//closest one from each camera as long as the distance is smaller than
		//merge_treshold and does a mean of the points.
		osg::ref_ptr<osg::Vec3Array> simple_2D_merge(
				std::vector<std::map<osg::Vec2, osg::Vec3> >* projection3d_array);

		//This merge method finds a white pixel starting from the bottom of the
		//image, going up in rows. Then searches for a pixel on the same row
		//in another camera, if the pixel is found and distance is smaller than
		//merge_treshold, it does a mean of the points.
		osg::ref_ptr<osg::Vec3Array> follow_path_2D_merge(
				std::vector < cv::Mat >* visited_pixels,
				std::vector<std::map<osg::Vec2, osg::Vec3> >* projection3d_array);

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

		int n_cameras;
		int n_frames;
		float merge_treshold;
};

#endif // SKELETONIZATION3D_H

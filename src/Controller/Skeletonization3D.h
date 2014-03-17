#ifndef SKELETONIZATION3D_H
#define SKELETONIZATION3D_H

#include "Skeletonization2D.h"
#include "PixelSearch.h"
#include "../View/RGBDCamera.h"
#include "../Model/PointCloud.h"

#include <vector>

#include <boost/shared_ptr.hpp>

class Skeletonization3D {
public:
	//Merge bone which are 5 cm apart from each other
	//Move bones inside the skin 2.5 cm since cameras indicate where is the
	//skin but not the bones.
	Skeletonization3D(const camVecT& camera_arr_, float merge_treshold_ = 0.02,
			float row_treshold_ = 0.01, float move_distance_ = 0.025);

	void generate_skeletonization();

	//Return an array of points, given a camera and a frame number
	//Important: This coordinates are relative to the camera
	const PointCloudPtr& get_simple_3d_projection(int cam_num, int frame_num);

	//Return an array of points, after merging all the camera views
	//Important: This coordinates are global
	const PointCloudPtr& get_merged_3d_projection(int frame_num);

	//Get a 2D skeleton frame
	const cv::Mat& get_2D_frame(int cam_num, int frame_num) const;

	const cv::Mat& get_2D_bin_frame(int cam_num, int frame_num) const;

	int get_n_frames() const;

	int get_d_rows() const;
	int get_d_cols() const;

protected:
private:
	//Merges several 2D images to a 3D complete image of a skeleton
	void merge_2D_skeletons(int frame_num);

	void do_3d_projection(int cam_num, int frame_num);

	//Internal method that does the hard work
	PointCloudPtr merge_2D_skeletons_impl(
			std::vector<const cv::Mat*>& skeletonized_frames, int frame_num);

	//Translate a set of given points, away from a camera a distance.
	//Points should be in CAMERA coordinates
	void translate_points_to_inside(PointCloudPtr& cloud, int cam_num) const;

	void get_global_coord_3d_projection(int cam_num, int frame_num,
			std::map<osg::Vec2, osg::Vec3>& projection3d) const;

	//merge_2D_skeletons_impl can call this two methods to actually do the
	//merging.

	//This method finds a white pixel starting from 0,0 and then calculates
	//distances to all white pixels in the other camera views, then takes the
	//closest one from each camera as long as the distance is smaller than
	//merge_treshold and does a mean of the points.
	PointCloudPtr simple_2D_merge(
			std::vector<std::map<osg::Vec2, osg::Vec3> >& projection3d_array);

	//This merge method finds a white pixel starting from the bottom of the
	//image, going up in rows. Then searches for a pixel on the same row
	//in another camera, if the pixel is found and distance is smaller than
	//merge_treshold, it does a mean of the points.
	PointCloudPtr follow_path_2D_merge(std::vector<cv::Mat>& visited_pixels,
			std::vector<std::map<osg::Vec2, osg::Vec3> >& projection3d_array);

	//Do not merge, just take all the points
	PointCloudPtr take_all_points_2D_merge(
			std::vector<std::map<osg::Vec2, osg::Vec3> >& projection3d_array);

	//Vector of Skeletonization class, there is one instance
	//for each camera
	std::vector<Skeletonization2DPtr> skel_arr;

	//Each Vec3Array is a cloud of points that represent a skeleton
	//in a given frame, viewed from a different camera
	std::vector<PointCloudPtr> skel2d_cam_array;

	//Each Vec3Array is a cloud of points that represent a skeleton
	//in a given frame, after merging the views from all the cameras
	std::vector<PointCloudPtr> skel3d_merged_array;

	//Reference to vector is much better than pointers
	//Pointers are evil
	const camVecT& camera_arr;

	int n_cameras;
	int n_frames;
	float merge_treshold;
	float row_treshold;
	float move_distance;
};

typedef boost::shared_ptr<Skeletonization3D> Skeletonization3DPtr;

#endif // SKELETONIZATION3D_H

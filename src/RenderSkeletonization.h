#ifndef RENDERSKELETONIZATION_H
#define RENDERSKELETONIZATION_H

#include "Skeletonization3D.h"
#include "RGBDCamera.h"

#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/Texture2D>

#include "boost/shared_ptr.hpp"

#include "iostream"

using std::cout;
using std::endl;

class RenderSkeletonization {
	public:
		RenderSkeletonization();
		RenderSkeletonization(
				std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr_,
				osg::ref_ptr<osg::Switch> skel_vis_switch_);
		virtual ~RenderSkeletonization();
		//Update/Draw the skeleton display every frame
		void update_dynamics(int disp_frame_no);
		void set_data(std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr_,
				osg::ref_ptr<osg::Switch> skel_vis_switch_);
	protected:
	private:
		//Delete skeleton nodes from previous frame
		void clean_scene();

		//Show a 3D skeleton projection given a frame number
		void display_3d_skeleon_cloud(int disp_frame_no);

		//Show a 3D skeleton projection given a frame number
		void display_3d_merged_skeleon_cloud(int disp_frame_no);

		//Show the 2D skeleton images of each camera given a frame number
		void display_2d_skeletons(int disp_frame_no);

		//Class that creates a skeleton from a given set of frames
		Skeletonization3D skeleton;

		//Pointer to the camera array
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr;

		//Root node of all skeleton related nodes
		osg::ref_ptr<osg::Switch> skel_vis_switch;

		//Direct access to skeleton nodes, ordered by cameras
		std::vector<osg::ref_ptr<osg::Group> > skel_group_array;

		//Direct access to skeleton nodes, ordered by cameras
		std::vector<osg::ref_ptr<osg::Group> > skel_group2D_array;

		//Direct access to skeleton nodes, ordered by cameras
		std::vector<osg::ref_ptr<osg::Group> > skel_group3D_array;

		//Merge skeleton group
		osg::ref_ptr<osg::Group> merged_group;

		bool display_merged;
};

#endif // RENDERSKELETONIZATION_H

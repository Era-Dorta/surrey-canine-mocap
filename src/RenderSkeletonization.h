#ifndef RENDERSKELETONIZATION_H
#define RENDERSKELETONIZATION_H

#include "Skeletonization3D.h"
#include "RGBDCamera.h"

#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Group>

#include "boost/shared_ptr.hpp"

#include "iostream"

using std::cout;
using std::endl;

class RenderSkeletonization
{
	public:
		RenderSkeletonization();
		RenderSkeletonization( std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_,
				osg::ref_ptr<osg::Switch> skel_vis_switch_);
		virtual ~RenderSkeletonization();
		//Update/Draw the skeleton display every frame
		void update_dynamics(int disp_frame_no);
		void set_data(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_,
				osg::ref_ptr<osg::Switch> skel_vis_switch_);
	protected:
	private:
		//Class that creates a skeleton from a given set of frames
		Skeletonization3D skeleton;
		//Pointer to the camera array
		std::vector<boost::shared_ptr<RGBD_Camera> >* camera_arr;
		osg::ref_ptr<osg::Switch> skel_vis_switch;
};

#endif // RENDERSKELETONIZATION_H

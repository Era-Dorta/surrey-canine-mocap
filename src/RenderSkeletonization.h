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
		RenderSkeletonization( std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_ );
		virtual ~RenderSkeletonization();
		//Update/Draw the skeleton display every frame
		void update_dynamics(int disp_frame_no);
		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_);
	protected:
	private:
		//Class that creates a skeleton from a given set of frames
		Skeletonization3D skeleton;
		//Pointer to the camera array
		std::vector<boost::shared_ptr<RGBD_Camera> >* camera_arr;
};

#endif // RENDERSKELETONIZATION_H

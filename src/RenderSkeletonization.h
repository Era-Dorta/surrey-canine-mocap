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
		void update_dynamics(int disp_frame_no);
		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_);
	protected:
	private:
		Skeletonization3D skeleton;
		osg::ref_ptr<osg::Vec3Array> vertices;
		std::vector<boost::shared_ptr<RGBD_Camera> >* camera_arr;
		std::vector<osg::ref_ptr<osg::Group> > cam_skel_nodes;
};

#endif // RENDERSKELETONIZATION_H

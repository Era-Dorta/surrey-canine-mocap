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
		RenderSkeletonization( std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr );
		virtual ~RenderSkeletonization();
		void update_dynamics(int disp_frame_no);
		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr);
		void set_node( osg::ref_ptr<osg::Group> new_skel_root );
	protected:
	private:
		Skeletonization3D skeleton;
		osg::ref_ptr<osg::Group> skel_root;
		osg::ref_ptr<osg::Vec3Array> vertices;
};

#endif // RENDERSKELETONIZATION_H

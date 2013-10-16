#ifndef RENDERSKELETONIZATION_H
#define RENDERSKELETONIZATION_H

#include "Skeletonization.h"
#include "RGBDCamera.h"

#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Group>

#include "boost/shared_ptr.hpp"

class RenderSkeletonization
{
	public:
		RenderSkeletonization();
		RenderSkeletonization( std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr );
		virtual ~RenderSkeletonization();
		void update_dynamics(void);
		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr);
		void set_node( osg::ref_ptr<osg::Group> new_skel_root );
		void draw();
	protected:
	private:
		std::vector < boost::shared_ptr<Skeletonization> > skel_arr;
		osg::ref_ptr<osg::Group> skel_root;
};

#endif // RENDERSKELETONIZATION_H

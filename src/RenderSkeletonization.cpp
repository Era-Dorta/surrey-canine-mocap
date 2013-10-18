#include "RenderSkeletonization.h"

RenderSkeletonization::RenderSkeletonization()
{

}

RenderSkeletonization::RenderSkeletonization(std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr)
{
	skeleton.set_cameras(camera_arr);
}

RenderSkeletonization::~RenderSkeletonization()
{
	//dtor
}

void RenderSkeletonization::set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr)
{
	skeleton.set_cameras(camera_arr);
}

void RenderSkeletonization::set_node( osg::ref_ptr<osg::Group> new_skel_root )
{
	skel_root = new_skel_root;
}

void RenderSkeletonization::update_dynamics( int disp_frame_no )
{
	/*
	//TODO Since only one camera, remove 1 children, to be changed later
	skel_root->removeChildren(0, 1);
	vertices = skel_arr[0]->get_points_for_frame(disp_frame_no);
	osg::ref_ptr<osg::Geode> skel_geode = new osg::Geode;
	skel_root->addChild(skel_geode);
	osg::ref_ptr<osg::Geometry> geom;

	//osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
	//Create geometry node for each cam to draw
	std::vector < boost::shared_ptr<Skeletonization> >::iterator it = skel_arr.begin();
	for(; it != skel_arr.end(); ++it) {
		geom = new osg::Geometry;
		//TODO, Check this out, best options seems to reserve many points and then
		//en each iteration tell it how many it should use
		geom->setVertexArray (vertices.get());
		//geom->setColorArray (colors.get());
		//geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
		skel_geode->addDrawable(geom.get());
	}
	*/
}

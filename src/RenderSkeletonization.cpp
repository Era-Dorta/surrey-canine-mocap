#include "RenderSkeletonization.h"

RenderSkeletonization::RenderSkeletonization()
{

}

RenderSkeletonization::RenderSkeletonization(std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr)
{
	for(int i = 0; i < camera_arr.size(); i++){
		boost::shared_ptr<Skeletonization> skel(new Skeletonization(camera_arr[i]->get_frames()));
		skel_arr.push_back(skel);
	}
}

RenderSkeletonization::~RenderSkeletonization()
{
	//dtor
}

void RenderSkeletonization::set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr)
{
	skel_arr.clear();
	//Just the second camera until I boost up the speed
	for(int i = 1; i < camera_arr.size() - 1; i++){
		boost::shared_ptr<Skeletonization> skel(new Skeletonization(camera_arr[i]->get_frames()));
		skel_arr.push_back(skel);
	}
}

void RenderSkeletonization::set_node( osg::ref_ptr<osg::Group> new_skel_root )
{
	skel_root = new_skel_root;

	osg::ref_ptr<osg::Geode> skel_geode = new osg::Geode;
	skel_root->addChild(skel_geode);

	osg::ref_ptr<osg::Geometry> geom;

	//Create geometry node for each cam to draw
	std::vector < boost::shared_ptr<Skeletonization> >::iterator it = skel_arr.begin();
	for(; it != skel_arr.end(); ++it) {
		geom = new osg::Geometry;
		//TODO, Check this out, best options seems to reserve many points and then
		//en each iteration tell it how many it should use
		geom->addPrimitiveSet( new DrawArrays(PrimitiveSet::POINTS, 0 ,4));
		skel_geode->addChild(geom);
	}
}

void RenderSkeletonization::draw()
{

}

#include "RenderSkeletonization.h"

RenderSkeletonization::RenderSkeletonization()
{
	camera_arr = NULL;
}

RenderSkeletonization::RenderSkeletonization(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_)
{
	set_cameras(camera_arr_);
}

RenderSkeletonization::~RenderSkeletonization()
{
	//dtor
}

void RenderSkeletonization::set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_)
{
	camera_arr = camera_arr_;
	skeleton.set_cameras(camera_arr);

	osg::ref_ptr<osg::Group> skel_group;
	osg::ref_ptr<osg::MatrixTransform> traslate_node;
	for(unsigned int i = 0; i < camera_arr->size(); i++){
		skel_group = new osg::Group;
		(*camera_arr)[i]->skel_vis_group->addChild(skel_group.get());

		traslate_node = new osg::MatrixTransform();
		traslate_node->setMatrix(osg::Matrix::scale(osg::Vec3(10.f, 0, 10.f)));
		skel_group->addChild(traslate_node.get());

		cam_skel_nodes.push_back(traslate_node.get());
	}
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

	osg::ref_ptr<osg::Geode> skel_geode;
	osg::ref_ptr<osg::Geometry> skel_geometry;
	osg::ref_ptr<osg::Group> cam_group;
	osg::ref_ptr<osg::Vec3Array> vertices;

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0)); //red

	for(unsigned int i = 0; i < camera_arr->size(); i++){
		cam_group = cam_skel_nodes[i];
		cam_group->removeChildren(0, 1);
		skel_geode = new osg::Geode;
		skel_geometry = new osg::Geometry;


		vertices = skeleton.get_points_for_camera(i, disp_frame_no);

		skel_geometry->setVertexArray (vertices.get());
		skel_geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
		skel_geometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
		skel_geode->addDrawable(skel_geometry.get());
		cam_group->addChild(skel_geode.get());
	}
}

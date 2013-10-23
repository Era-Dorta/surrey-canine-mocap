#include "RenderSkeletonization.h"

RenderSkeletonization::RenderSkeletonization()
{
	camera_arr = NULL;
}

RenderSkeletonization::RenderSkeletonization(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_,
		osg::ref_ptr<osg::Switch> skel_vis_switch_)
{
	set_data(camera_arr_, skel_vis_switch_);
}

RenderSkeletonization::~RenderSkeletonization()
{
	//dtor
}

void RenderSkeletonization::set_data(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_,
		osg::ref_ptr<osg::Switch> skel_vis_switch_)
{
	camera_arr = camera_arr_;
	skeleton.set_cameras(camera_arr);
	skel_vis_switch = skel_vis_switch_;

	osg::ref_ptr<osg::Group> skel_group;
	for(unsigned int i = 0; i < camera_arr->size(); i++){
		skel_group = new osg::Group;
		skel_vis_switch->addChild(skel_group.get());
		//(*camera_arr)[i]->skel_vis_group->addChild(skel_group.get());
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
	osg::ref_ptr<osg::Vec3Array> vertices;

	osg::ref_ptr<osg::Group> skel_group;

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0)); //red

	//Uncoment this to give the lines some width
	//osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth();
	//linewidth->setWidth(2.0f);

	//Draw a red cloud of points, where each point represents a small part of a bone
	for(unsigned int i = 0; i < camera_arr->size(); i++){
		skel_geode = new osg::Geode;
		skel_geometry = new osg::Geometry;
		skel_group = static_cast<osg::Group*>(skel_vis_switch->getChild(i));

		skel_group->removeChildren(0, skel_group->getNumChildren());

		vertices = skeleton.get_points_for_camera(i, disp_frame_no);

		skel_geometry->setVertexArray (vertices.get());
		skel_geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
		//Should be POINTS but this is better to see errors
		skel_geometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
		skel_geode->addDrawable(skel_geometry.get());
		//skel_geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

		skel_group->addChild(skel_geode.get());
	}

	osg::ref_ptr<osg::MatrixTransform> trans_matrix;

	//Define a quad
	vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(-0.5f, -0.5f,2.f));
	vertices->push_back(osg::Vec3(0.5f, -0.5f, 2.f));
	vertices->push_back(osg::Vec3(0.5f, 0.5f, 2.f));
	vertices->push_back(osg::Vec3(-0.5f, 0.5f, 2.f));

	//Understanding the texture mapping
	//In a standard Y right Y up axis, quad was defined as
	// 3, 2
	// 0, 1
	//While texture mapping world for a quad in the same axis is defined as
	// [0,1] [1,1]
	// [0,0] [1,0]
	//Then the coordinates of the texture has to follow the order of the
	//definition of the quad
	osg::ref_ptr<osg::Vec2Array> tc = new osg::Vec2Array;
	tc->push_back( osg::Vec2( 0.f, 0.f ) );
	tc->push_back( osg::Vec2( 1.f, 0.f ) );
	tc->push_back( osg::Vec2( 1.f, 1.f ) );
	tc->push_back( osg::Vec2( 0.f, 1.f ) );

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back( osg::Vec3(0.0f, 0.0f, -1.0f) );

	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
	quad->setVertexArray( vertices.get() );
	quad->setNormalArray( normals.get() );
	quad->setNormalBinding( osg::Geometry::BIND_OVERALL );
	quad->setTexCoordArray( 0, tc.get() );
	quad->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );

	cv::Mat* cvImg = skeleton.skel_arr[1]->get_frame(disp_frame_no);

	osg::ref_ptr<osg::Image> osgImage = new osg::Image;
	osgImage->setImage(cvImg->cols,cvImg->rows, 3,
	                           GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, cvImg->data,
	                           osg::Image::NO_DELETE);

	//osg::ref_ptr<osg::Image> osgImage = osgDB::readImageFile( "../../data/test.jpg" );

	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
	tex->setImage( osgImage.get() );

	osg::ref_ptr<osg::Geode> root = new osg::Geode;
	root->addDrawable( quad.get() );
	root->getOrCreateStateSet()->setTextureAttributeAndModes( 0, tex.get() );

	skel_group = static_cast<osg::Group*>(skel_vis_switch->getChild(0));
	skel_group->addChild(root.get());

	/*osg::ref_ptr<osg::Group> skel_group;
	for(unsigned int i = 0; i < camera_arr->size(); i++){
		skel_geode = new osg::Geode;
		skel_geometry = new osg::Geometry;
		skel_geometry->setTexCoordArray( 0, tc.get() );
		skel_group = static_cast<osg::Group*>(skel_vis_switch->getChild(i));
		trans_matrix = new osg::MatrixTransform;
		trans_matrix->setMatrix(osg::Matrix::translate(osg::Vec3(10.f, 0, 10.f)));

		vertices = skeleton.get_points_for_camera(i, disp_frame_no);

		skel_geometry->setVertexArray (vertices.get());
		skel_geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
		//Should be POINTS but this is better to see errors
		skel_geometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
		skel_geode->addDrawable(skel_geometry.get());
		//skel_geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

		skel_group->addChild(skel_geode.get());
	}*/
}

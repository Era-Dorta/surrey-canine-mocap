#include "RenderSkeletonization.h"

RenderSkeletonization::RenderSkeletonization() :
			display_merged(true) {

}

RenderSkeletonization::RenderSkeletonization(
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr_,
		osg::ref_ptr<osg::Switch> skel_vis_switch_) :
			display_merged(true) {
	set_data(camera_arr_, skel_vis_switch_);
}

RenderSkeletonization::~RenderSkeletonization() {
	//dtor
}

void RenderSkeletonization::set_data(
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr_,
		osg::ref_ptr<osg::Switch> skel_vis_switch_) {
	//Save arguments
	camera_arr = camera_arr_;
	skeleton.set_cameras(camera_arr);
	skel_vis_switch = skel_vis_switch_;

	//In case this is not first call, do a clean up
	skel_group_array.clear();
	skel_group2D_array.clear();
	skel_group3D_array.clear();
	skel_vis_switch->removeChildren(0, skel_vis_switch->getNumChildren());

	//Set up the basics nodes
	for (unsigned int i = 0; i < camera_arr.size(); i++) {
		//Create main skeleton group for this camera and save it in an vector
		osg::ref_ptr<osg::Group> skel_group = new osg::Group;
		skel_vis_switch->addChild(skel_group.get(), true);
		skel_group_array.push_back(skel_group.get());

		//Create 2D skeleton group for this camera, put it under skeleton group
		//and save it in an vector
		osg::ref_ptr<osg::Group> aux_group = new osg::Group;
		skel_group->addChild(aux_group.get());
		skel_group2D_array.push_back(aux_group.get());

		//Get this camera transformation
		osg::ref_ptr<osg::MatrixTransform> cam_transform =
				new osg::MatrixTransform;
		//The original camera node is not used to keep the scene graph simpler
		cam_transform->setMatrix(camera_arr[i]->cam_pose_xform->getMatrix());

		//Add it as child of main skeleton group
		skel_group->addChild(cam_transform.get());

		//Create 3D skeleton group under the camera transformation
		aux_group = new osg::Group;
		cam_transform->addChild(aux_group.get());
		skel_group3D_array.push_back(aux_group.get());
	}

	merged_group = new osg::Group;
	skel_vis_switch->addChild(merged_group.get(), true);
}

void RenderSkeletonization::update_dynamics(int disp_frame_no) {
	clean_scene();

	//display_2d_skeletons(disp_frame_no);

	display_3d_skeleon_cloud(disp_frame_no);

	display_3d_merged_skeleon_cloud(disp_frame_no);
}

void RenderSkeletonization::clean_scene() {
	for (unsigned int i = 0; i < skel_group2D_array.size(); i++) {
		skel_group2D_array[i]->removeChildren(0,
				skel_group2D_array[i]->getNumChildren());
	}

	for (unsigned int i = 0; i < skel_group3D_array.size(); i++) {
		skel_group3D_array[i]->removeChildren(0,
				skel_group3D_array[i]->getNumChildren());
	}

	merged_group->removeChildren(0, merged_group->getNumChildren());
}

void RenderSkeletonization::display_3d_skeleon_cloud(int disp_frame_no) {
	osg::ref_ptr<osg::Geode> skel_geode;
	osg::ref_ptr<osg::Geometry> skel_geometry;
	osg::ref_ptr<osg::Vec3Array> vertices;

	//Draw a red cloud of boxes, where each box represents a small part of a bone
	for (unsigned int i = 0; i < camera_arr.size(); i++) {
		skel_geode = new osg::Geode;
		skel_geometry = new osg::Geometry;

		vertices = skeleton.get_simple_3d_projection(i, disp_frame_no);

		for (unsigned int j = 0; j < vertices->size(); j++) {
			osg::ref_ptr<osg::ShapeDrawable> shape1 = new osg::ShapeDrawable;
			shape1->setShape(
					new osg::Box((*vertices)[j], 0.005f, 0.005f, 0.005f));
			shape1->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0)); //red
			skel_geode->addDrawable(shape1.get());
		}

		skel_group3D_array[i]->addChild(skel_geode.get());
	}
}

void RenderSkeletonization::display_3d_merged_skeleon_cloud(int disp_frame_no) {
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0)); //blue

	//Draw a blue cloud of squares, where each square represents a small part of a bone
	osg::ref_ptr<osg::Geode> skel_geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> skel_geometry = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> vertices = skeleton.get_merged_3d_projection(
			disp_frame_no);

	for (unsigned int j = 0; j < vertices->size(); j++) {
		osg::ref_ptr<osg::ShapeDrawable> shape1 = new osg::ShapeDrawable;
		shape1->setShape(new osg::Box((*vertices)[j], 0.005f, 0.005f, 0.005f));
		shape1->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0)); //green
		skel_geode->addDrawable(shape1.get());
	}

	merged_group->addChild(skel_geode.get());
}

void RenderSkeletonization::display_2d_skeletons(int disp_frame_no) {
	int rows = camera_arr[0]->get_d_rows();
	int cols = camera_arr[0]->get_d_cols();
	float ratio_x = cols / (float) rows;

	//Define a quad
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(0.f, 0.f, 2.f));
	vertices->push_back(osg::Vec3(ratio_x, 0.f, 2.f));
	vertices->push_back(osg::Vec3(ratio_x, 1.f, 2.f));
	vertices->push_back(osg::Vec3(0.f, 1.f, 2.f));

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
	tc->push_back(osg::Vec2(0.f, 0.f));
	tc->push_back(osg::Vec2(ratio_x, 0.f));
	tc->push_back(osg::Vec2(ratio_x, 1.f));
	tc->push_back(osg::Vec2(0.f, 1.f));

	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back(osg::Vec3(0.0f, 0.0f, -1.0f));
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0)); //green

	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
	quad->setVertexArray(vertices.get());
	quad->setNormalArray(normals.get());
	quad->setNormalBinding(osg::Geometry::BIND_OVERALL);
	quad->setColorArray(colors, osg::Array::BIND_OVERALL);
	quad->setTexCoordArray(0, tc.get());
	quad->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

	for (unsigned int i = 0; i < camera_arr.size(); i++) {
		const cv::Mat* cvImg = skeleton.get_2D_frame(i, disp_frame_no);

		osg::ref_ptr<osg::Image> osgImage = new osg::Image;
		osgImage->setImage(cvImg->cols, cvImg->rows, 3,
		GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_BYTE, cvImg->data,
				osg::Image::NO_DELETE);

		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
		tex->setImage(osgImage.get());

		osg::ref_ptr<osg::Geode> skel2d_geode = new osg::Geode;
		skel2d_geode->addDrawable(quad.get());
		skel2d_geode->getOrCreateStateSet()->setTextureAttributeAndModes(0,
				tex.get());

		osg::ref_ptr<osg::MatrixTransform> trans_matrix =
				new osg::MatrixTransform;
		trans_matrix->setMatrix(
				osg::Matrix::translate(osg::Vec3(1.4f * i - 2.f, -0.5f, 0.f)));
		trans_matrix->addChild(skel2d_geode.get());

		skel_group2D_array[i]->addChild(trans_matrix.get());
	}
}

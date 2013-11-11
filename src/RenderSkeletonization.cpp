#include "RenderSkeletonization.h"

RenderSkeletonization::RenderSkeletonization() :
			display_merged(true) {
	joint_colour = osg::Vec4(0.5f, 0.5f, 0.5f, 1.0); //Grey
	bone_colour = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0); //Blue
	selection_colour = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0); //White
}

RenderSkeletonization::~RenderSkeletonization() {
	//dtor
}

void RenderSkeletonization::set_data(
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr_,
		osg::ref_ptr<osg::Switch> skel_vis_switch_,
		osg::ref_ptr<osg::Switch> skel_fitting_switch_) {
	//Save arguments
	camera_arr = camera_arr_;
	skel_vis_switch = skel_vis_switch_;
	skel_fitting_switch = skel_fitting_switch_;

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

	skel_fitting_switch->removeChildren(0,
			skel_fitting_switch->getNumChildren());
}

void RenderSkeletonization::display_3d_skeleon_cloud(int disp_frame_no,
		Skeletonization3D& skeleton) {
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
			shape1->setColor(osg::Vec4(camera_arr[i]->getVisColour(), 1.0f));
			skel_geode->addDrawable(shape1.get());
		}

		skel_group3D_array[i]->addChild(skel_geode.get());
	}
}

void RenderSkeletonization::display_3d_merged_skeleon_cloud(int disp_frame_no,
		Skeletonization3D& skeleton) {
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
		shape1->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0)); //Red
		skel_geode->addDrawable(shape1.get());
	}

	merged_group->addChild(skel_geode.get());
}

void RenderSkeletonization::display_2d_skeletons(int disp_frame_no,
		Skeletonization3D& skeleton) {
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

void RenderSkeletonization::draw_joints(
		osg::ref_ptr<osg::Vec3Array> joint_array) {
	for (unsigned int i = 0; i < joint_array->size(); i++) {
		osg::Vec3 joint_position = (*joint_array)[i];
		osg::ref_ptr<osg::MatrixTransform> selectionBox = createSelectionBox();
		selectionBox->setMatrix(
				osg::Matrix::scale(0.02, 0.02, 0.02)
						* osg::Matrix::translate(joint_position));

		skel_fitting_switch->addChild(selectionBox.get(), true);
	}
}

osg::ref_ptr<osg::MatrixTransform> RenderSkeletonization::createSelectionBox() {
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::ShapeDrawable> box_shape;
	box_shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1.1f));
	box_shape->setColor(joint_colour);

	geode->getOrCreateStateSet()->setMode( GL_LIGHTING,
			osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	geode->addDrawable(box_shape);
	osg::ref_ptr<osg::MatrixTransform> selectionBox = new osg::MatrixTransform;

	selectionBox->addChild(geode.get());

	return selectionBox;
}

void RenderSkeletonization::change_colour_when_selected(
		osg::ref_ptr<osg::MatrixTransform> selected_point,
		bool point_selected) {
	osg::ref_ptr<osg::ShapeDrawable> obj_shape;
	osg::ref_ptr<osg::Geode> obj_geode;

	obj_geode = static_cast<osg::Geode*>(selected_point->getChild(0));
	obj_shape = static_cast<osg::ShapeDrawable*>(obj_geode->getDrawable(0));
	if (point_selected) {
		obj_shape->setColor(selection_colour);
	} else {
		obj_shape->setColor(joint_colour);
	}
}

void RenderSkeletonization::evaluate_children(NODE* node, MOCAPHEADER& header,
		int current_frame) {
	//cout << "drawing children" << endl;

	glPushMatrix();
	glTranslatef(node->offset[0] + node->froset[current_frame][0],
			node->offset[1] + node->froset[current_frame][1],
			node->offset[2] + node->froset[current_frame][2]);

	glRotatef(node->euler[0], (float) header.euler[0][0],
			(float) header.euler[0][1], (float) header.euler[0][2]);
	glRotatef(node->euler[1], (float) header.euler[1][0],
			(float) header.euler[1][1], (float) header.euler[1][2]);
	glRotatef(node->euler[2], (float) header.euler[2][0],
			(float) header.euler[2][1], (float) header.euler[2][2]);

	glRotatef(node->freuler[current_frame][0], (float) header.euler[0][0],
			(float) header.euler[0][1], (float) header.euler[0][2]);
	glRotatef(node->freuler[current_frame][1], (float) header.euler[1][0],
			(float) header.euler[1][1], (float) header.euler[1][2]);
	glRotatef(node->freuler[current_frame][2], (float) header.euler[2][0],
			(float) header.euler[2][1], (float) header.euler[2][2]);

	glBegin(GL_LINES);
		glColor3f(node->colour[0], node->colour[1], node->colour[2]);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(node->length[0] * node->scale[current_frame],
				node->length[1] * node->scale[current_frame],
				node->length[2] * node->scale[current_frame]);
	glEnd();

	if (node->children) {
		for (int i = 0; i < node->noofchildren; i++)
			evaluate_children(node->children[i], header, current_frame);
	}

	glPopMatrix();
}

void RenderSkeletonization::AddCylinderBetweenPoints(osg::Vec3 StartPoint,
		osg::Vec3 EndPoint, float radius, osg::Vec4 CylinderColor,
		osg::Group *pAddToThisGroup) {
	osg::Vec3 center;
	float height;

	osg::ref_ptr<osg::Cylinder> cylinder;
	osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable;
	osg::ref_ptr<osg::Material> pMaterial;
	osg::ref_ptr<osg::Geode> geode;

	height = (StartPoint - EndPoint).length();
	center = osg::Vec3((StartPoint.x() + EndPoint.x()) / 2,
			(StartPoint.y() + EndPoint.y()) / 2,
			(StartPoint.z() + EndPoint.z()) / 2);

	// This is the default direction for the cylinders to face in OpenGL
	osg::Vec3 z = osg::Vec3(0, 0, 1);

	// Get diff between two points you want cylinder along
	osg::Vec3 p = (StartPoint - EndPoint);

	// Get CROSS product (the axis of rotation)
	osg::Vec3 t = z ^ p;

	// Get angle. length is magnitude of the vector
	double angle = acos((z * p) / p.length());

	//   Create a cylinder between the two points with the given radius
	cylinder = new osg::Cylinder(center, radius, height);
	cylinder->setRotation(osg::Quat(angle, osg::Vec3(t.x(), t.y(), t.z())));

	//   A geode to hold our cylinder
	geode = new osg::Geode;
	cylinderDrawable = new osg::ShapeDrawable(cylinder);
	geode->addDrawable(cylinderDrawable);

	//   Set the color of the cylinder that extends between the two points.
	pMaterial = new osg::Material;
	pMaterial->setDiffuse(osg::Material::FRONT, CylinderColor);
	geode->getOrCreateStateSet()->setAttribute(pMaterial,
			osg::StateAttribute::OVERRIDE);

	//   Add the cylinder between the two points to an existing group
	pAddToThisGroup->addChild(geode);
}

osg::Vec3 RenderSkeletonization::add_sphere(intersecIte intersection) {
	osg::BoundingBox bb = intersection->drawable->getBound();
	osg::Vec3 worldCenter = bb.center()
			* osg::computeLocalToWorld(intersection->nodePath);

	osg::ref_ptr<osg::MatrixTransform> selectionBox = createSelectionBox();
	selectionBox->setMatrix(
			osg::Matrix::scale(bb.xMax() + 0.005 - bb.xMin(),
					bb.yMax() + 0.005 - bb.yMin(),
					bb.zMax() + 0.005 - bb.zMin())
					* osg::Matrix::translate(worldCenter));

	skel_fitting_switch->addChild(selectionBox.get(), true);

	//Return global coordinates of the point
	return osg::Vec3() * selectionBox->getMatrix();
}

osg::Vec3 RenderSkeletonization::move_sphere(intersecIte intersection,
		osg::ref_ptr<osg::MatrixTransform> obj) {
	osg::BoundingBox bb = intersection->drawable->getBound();
	osg::Vec3 worldCenter = bb.center()
			* osg::computeLocalToWorld(intersection->nodePath);

	obj->setMatrix(
			osg::Matrix::scale(bb.xMax() + 0.005 - bb.xMin(),
					bb.yMax() + 0.005 - bb.yMin(),
					bb.zMax() + 0.005 - bb.zMin())
					* osg::Matrix::translate(worldCenter));

	//Return global coordinates of the point
	return osg::Vec3() * obj->getMatrix();
}

osg::Vec3 RenderSkeletonization::move_sphere(osg::Vec3& move_vec,
		osg::Camera* cam, osg::MatrixTransform* obj) {

	osg::Vec3 aux_vec = osg::Matrixd::inverse(cam->getProjectionMatrix())
			* move_vec * 0.01;
	osg::Matrix aux_matrix = osg::Matrix::translate(aux_vec);
	aux_matrix = cam->getViewMatrix() * aux_matrix
			* cam->getInverseViewMatrix();
	osg::Matrix new_matrix = obj->getMatrix() * aux_matrix;

	return osg::Vec3() * new_matrix;
}

int RenderSkeletonization::obj_belong_skel(osg::MatrixTransform* selected_obj) {
	for (unsigned int i = 0; i < skel_fitting_switch->getNumChildren(); i++) {
		if (selected_obj == skel_fitting_switch->getChild(i)) {
			return i;
		}
	}
	return -1;
}

void RenderSkeletonization::draw_bone(osg::Vec3& bone_start,
		osg::Vec3& bone_end) {

	AddCylinderBetweenPoints(bone_start, bone_end, 0.01f, bone_colour,
			skel_fitting_switch);
}

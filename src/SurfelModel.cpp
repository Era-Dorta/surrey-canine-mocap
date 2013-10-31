/*
 * SurfelModel.cpp
 *
 *  Created on: Aug 6, 2013
 *      Author: cm00215
 */

#include "SurfelModel.h"

using std::cout;
using std::endl;

SurfelModel::SurfelModel() {

	surfel_geode = new osg::Geode;

	//Allocate arrays:
	//------------------------
	surfel_vertices = new osg::Vec3Array;	//(5);
	surfel_normals = new osg::Vec3Array;	//(5);
	surfel_colours = new osg::Vec4Array;	//(5);
	surfel_attributes = new osg::Vec3Array;	//(5);
	//------------------------

	//Setup geometry node:
	//------------------------
	surfel_geometry = new osg::Geometry;
	surfel_geometry->setDataVariance(osg::Object::DYNAMIC);
	surfel_geometry->setVertexArray(surfel_vertices);
	surfel_geometry->setNormalArray(surfel_normals);
	surfel_geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

	surfel_geometry->setColorArray(surfel_colours.get());
	surfel_geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	//Note: use index not assigned to a built in attribute
	//(see http://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/attributes.php)
	surfel_geometry->setVertexAttribArray(6, surfel_attributes.get());
	surfel_geometry->setVertexAttribBinding(6, osg::Geometry::BIND_PER_VERTEX);
	surfel_geometry->addPrimitiveSet(
			new osg::DrawArrays(GL_POINTS, 0, surfel_vertices->size()));
	//Use VBO to improve performance:
	surfel_geometry->setUseVertexBufferObjects(true);
	surfel_geometry->setUseDisplayList(false);

	surfel_geode->addDrawable(surfel_geometry);
	//------------------------

	//Enable splat rendering:
	//------------------------
	enable_splat_rendering(surfel_geode);
	//------------------------

	//DEBUG TEST: Add some points
	//------------------------

//	pushback_surfel(osg::Vec3(0,0,0.5), osg::Vec3(0.1,0.5,1.0), osg::Vec4(0,1,0,1), 0.05f, -1, -1);
//	pushback_surfel(osg::Vec3(0,0,0.7), osg::Vec3(0.1,0.5,1.0), osg::Vec4(0,1,0,1), 0.05f, -1, -1);
//	pushback_surfel(osg::Vec3(0.3,0.05,0.2), osg::Vec3(0.2,0.2,0.7), osg::Vec4(0,1,0,1), 0.05f, -1, -1);
//	pushback_surfel(osg::Vec3(0,0,0), osg::Vec3(0.1,0.2,0.5), osg::Vec4(0,1,0,1), 0.2f, -1, -1);
//	pushback_surfel(osg::Vec3(0,0,0), osg::Vec3(0.1,0.2,0.5), osg::Vec4(0,1,0,1), 0.2f, -1, -1);
//	pushback_surfel(osg::Vec3(0.05,0,-0.8), osg::Vec3(0.4,0.2,0.6), osg::Vec4(0,1,0,1), 0.05f, -1, -1);
//	pushback_surfel(osg::Vec3(0,0,0), osg::Vec3(0.1,0.2,0.5), osg::Vec4(0,1,0,1), 0.2f, -1, -1);
//	pushback_surfel(osg::Vec3(0,0,0.8), osg::Vec3(0.1,0.5,1.0), osg::Vec4(0,1,0,1), 0.05f, -1, -1);
//	pushback_surfel(osg::Vec3(0,0,0), osg::Vec3(0.1,0.6,0.1), osg::Vec4(0,1,0,1), 0.2f, -1, -1);
//	pushback_surfel(osg::Vec3(0,0,0), osg::Vec3(0.1,0.2,0.5), osg::Vec4(0,1,0,1), 0.2f, -1, -1);
//	refresh_primative_set();

	//------------------------

	remove_invalid_surfels();

}

SurfelModel::~SurfelModel() {
}

void SurfelModel::pushback_surfel(osg::Vec3 vertex, osg::Vec3 normal,
		osg::Vec4 colour, float radius, int time_stamp, int label) {
	surfel_vertices->push_back(vertex);
	surfel_normals->push_back(normal);
	surfel_colours->push_back(colour);
	surfel_attributes->push_back(osg::Vec3(radius, time_stamp, label));
}

void SurfelModel::set_surfel(int index, osg::Vec3 vertex, osg::Vec3 normal,
		osg::Vec4 colour, float radius, int time_stamp, int label) {
	(*surfel_vertices)[index] = vertex;
	(*surfel_normals)[index] = normal;
	(*surfel_colours)[index] = colour;
	(*surfel_attributes)[index] = osg::Vec3(radius, time_stamp, label);
}

void SurfelModel::copy_surfel(int src_idx, int dst_idx) {
	(*surfel_vertices)[dst_idx] = (*surfel_vertices)[src_idx];
	(*surfel_normals)[dst_idx] = (*surfel_normals)[src_idx];
	(*surfel_colours)[dst_idx] = (*surfel_colours)[src_idx];
	(*surfel_attributes)[dst_idx] = (*surfel_attributes)[src_idx];
}

void SurfelModel::refresh_primative_set(void) {
	surfel_geometry->removePrimitiveSet(0, 1);
	surfel_geometry->addPrimitiveSet(
			new osg::DrawArrays(GL_POINTS, 0, surfel_vertices->size()));
}

void SurfelModel::remove_invalid_surfels(void) {

	//cout << "Removing invalid surfels" << endl;

	//If no points, just return:
	if (surfel_vertices->size() == 0) {
		return;
	}

	//Sort into valid surfels followed by invalid:
	//-------------------

	unsigned int forward_pos = 0;
	unsigned int backward_pos = surfel_vertices->size() - 1;
	while (forward_pos < surfel_vertices->size()) {
		//DEBUG:
		//cout << "forward pos: " << forward_pos << ", backward pos: " << backward_pos << endl;

		//If point is invalid
		if ((*surfel_vertices)[forward_pos].z() == 0)//<--NB set the condition here
				{
			while ((*surfel_vertices)[backward_pos].z() == 0)	//<-- and here
			{
				backward_pos--;
			}
			if (backward_pos <= forward_pos) {
				break;
			} else {
				copy_surfel(backward_pos, forward_pos);
				forward_pos++;
				backward_pos--;
			}

		} else {
			forward_pos++;
		}
	}
	//Note: forward_pos ends up two past the index of the final valid element
	//-------------------

	//Resize array to remove the invalid points:
	//-------------------
	surfel_vertices->resize(forward_pos - 1);
	surfel_normals->resize(forward_pos - 1);
	surfel_colours->resize(forward_pos - 1);
	surfel_attributes->resize(forward_pos - 1);
	refresh_primative_set();
	//-------------------

//	//DEBUG:
//	cout << "Final forward pos: " << forward_pos << ", backward pos: " << backward_pos << endl;
//	for(int i = 0; i < surfel_vertices->size(); i++)
//	{
//		cout << i << "\t-\t" << (*surfel_vertices)[i].z() << endl;
//	}

}

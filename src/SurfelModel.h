/*
 * SurfelModel.h
 *
 *  Created on: Aug 6, 2013
 *      Author: cm00215
 */

#ifndef SURFELMODEL_H_
#define SURFELMODEL_H_

#include <string>
#include <vector>
#include <fstream>

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include <osgText/Text>
#include <osgText/Font>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/PolygonMode>
#include <osgViewer/CompositeViewer>
#include <osg/Geometry>
#include <osgUtil/SmoothingVisitor>

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#include "SimpleTimer.h"

#include "CudaVec.h"

//For containing (unstructured) surfel models

#include "MiscUtils.h"

class SurfelModel {
	public:
		SurfelModel();
		virtual ~SurfelModel();

		//TODO copy constructor, assignment etc

		void pushback_surfel(osg::Vec3 vertex, osg::Vec3 normal,
				osg::Vec4 colour, float radius, int time_stamp, int label);
		void set_surfel(int index, osg::Vec3 vertex, osg::Vec3 normal,
				osg::Vec4 colour, float radius, int time_stamp, int label);
		void refresh_primative_set(void);
		void remove_invalid_surfels(void);
		void copy_surfel(int src_idx, int dst_idx);

		//Geode:
		osg::ref_ptr<osg::Geode> surfel_geode;

	private:

		osg::Vec4 vis_colour;

		osg::ref_ptr<osg::Geometry> surfel_geometry;
		osg::ref_ptr<osg::Vec3Array> surfel_vertices;
		osg::ref_ptr<osg::Vec3Array> surfel_normals;
		osg::ref_ptr<osg::Vec4Array> surfel_colours;
		osg::ref_ptr<osg::Vec3Array> surfel_attributes;

};

#endif /* SURFELMODEL_H_ */

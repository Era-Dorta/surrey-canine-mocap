/*
 * DepthMapPoly.h
 *
 *  Created on: 11 Jul 2013
 *      Author: cm00215
 */

#ifndef DEPTHMAPPOLY_H_
#define DEPTHMAPPOLY_H_

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

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#include "SimpleTimer.h"

#include "CudaVec.h"

#include "MiscUtils.h"

//inline float3 osgvec_2_float3(osg::Vec3 vec)
//{
//	float3 temp;
//	temp.x = vec.x();
//	temp.y = vec.y();
//	temp.z = vec.z();
//	return temp;
//}

class DepthMapPoly {
public:
	DepthMapPoly();
	void init(int _width, int _height, osg::Vec3 _vis_colour);
	virtual ~DepthMapPoly();
	void polygonise_depth_map(cv::Mat* depth_map, cv::Mat* rgb_image, cv::Mat K,
			osg::Vec3 vis_colour, bool with_colour, float alpha);
	//Geode:
	osg::ref_ptr<osg::Geode> depth_poly_geode;

private:
	osg::Geode* create_depth_map_geode();
	//void polygonise_depth_map_cv(cv::Mat* depth_map, cv::Mat K, float discon_thresh);

	int rows;
	int cols;

	osg::Vec4 vis_colour;

	//Geometry of the triangulated depth map:
	osg::ref_ptr<osg::Geode> dm_geode;

	osg::ref_ptr<osg::Geometry> dm_geometry;
	osg::ref_ptr<osg::Vec3Array> dm_vertices;
	osg::ref_ptr<osg::Vec3Array> dm_normals;
	osg::ref_ptr<osg::Vec4Array> dm_colours;
	osg::ref_ptr<osg::DrawElementsUInt> dm_vert_indices;

};

#endif /* DEPTHMAPPOLY_H_ */

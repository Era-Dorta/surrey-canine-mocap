/*
 * MiscUtils.h
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#ifndef MISCUTILS_H_
#define MISCUTILS_H_

#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>
#include <osgText/Text3D>
#include <osgText/Font3D>
#include <osgText/Font>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/PolygonMode>
#include <osgViewer/CompositeViewer>
#include <osg/Geometry>
#include <osg/GraphicsContext>
#include <osg/BlendFunc>
#include <osg/LineWidth>
#include <osg/Program>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string/predicate.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/shared_ptr.hpp"

#include "opencv2/opencv.hpp"

#include "CudaVec.h"

namespace MiscUtils {

	void get_file_names(std::string path_in,
			std::vector<std::string>* file_names);

	void get_dir_names(std::string path_in,
			std::vector<std::string>* file_names);

	osg::Matrix cvmat4x4_2_osgmat(cv::Mat input);

	osg::Matrix3 cvmat3x3_2_osgmat(cv::Mat input);

	osgText::Text* create_text(const osg::Vec3& pos, const std::string& content,
			float size);

	osg::Geode* create_axes(float over_length = 1);

	osg::Group* create_3D_label(std::string text, osg::Vec3 offset,
			osg::Vec3 colour);

	//Use OpenCV to convert from an HSV value to and RGB value
	//(useful for easily specifying a more varied spectrum of colours for visualization)
	osg::Vec3 hsv_2_rgb(osg::Vec3 hsv_in);

	float3 osgvec_2_float3(osg::Vec3 vec);

	//Indicates whether to use normals for the shader or camera colour
	extern bool use_normal_shader;

	//Enable custom shaders for splat rendering of geometry
	void enable_splat_rendering(osg::ref_ptr<osg::Geode> geometry_in);

	void surfel_ply_write(osg::ref_ptr<osg::Geode> geom_in,
			std::string file_name);

	osg::Geode* surfel_ply_read(std::string file_name);

	//TODO Maybe it would be better to put this method in some abstract class
	//that Skeletonization classes would inherit from. Lest actually see how much
	//they turn out to have in common

	//Given a uchar 1 channel image, and a start points it returns true and a position
	//if it finds a white pixel next to it. False otherwise.
	bool get_neighbor_white_pixel(cv::Mat& img, int i_row, int i_col,
			int &res_row, int &res_col);
}

#endif /* MISCUTILS_H_ */

/*
 * RGBDFrame.h
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#ifndef RGBDFRAME_H_
#define RGBDFRAME_H_

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

#include "boost/filesystem.hpp"
#include "boost/algorithm/string/predicate.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/shared_ptr.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

class RGBD_Frame {
	public:
		RGBD_Frame();
		virtual ~RGBD_Frame();
		int frame_num;
		cv::Mat depth_img;
		cv::Mat rgb_img;
		double depth_timestamp_device;
		double rgb_timestamp_device;
		double depth_timestamp_global;
		double rgb_timestamp_global;
};

#endif /* RGBDFRAME_H_ */

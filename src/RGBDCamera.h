/*
 * RGBDCamera.h
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#ifndef RGBDCAMERA_H_
#define RGBDCAMERA_H_

#include "MiscUtils.h"
#include "RGBDFrame.h"
//#include "DepthMapPoly.h"
#include "DepthMapSurfel.h"
#include "SimpleTimer.h"
#include "RenderPOV.h"
#include "Skeletonization.h"

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
#include <osg/LineWidth>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string/predicate.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/shared_ptr.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

class RGBD_Camera
{
public:
	RGBD_Camera(std::string dataset_path, std::string cam_name);
	virtual ~RGBD_Camera();
	int get_first_frame_num(void);
	int get_last_frame_num(void);
	int get_d_rows(void);
	int get_d_cols(void);
	osg::Vec3 get_vis_colour(void);
	cv::Mat get_K_rgb(void);
	cv::Mat get_T_rgb(void);
	cv::Mat* get_depth_map(int frame_num);
	cv::Mat* get_rgb_image(int frame_num);
	std::map<int, RGBD_Frame>* get_frames();
	std::string get_cam_name(void);
	void get_surface_paths(int frame_num, std::vector< std::vector<cv::Point> >& surface_paths);
	void get_surface_paths_3d(int frame_num, std::vector< std::vector<float3> >& surface_paths_3d);
	osg::ref_ptr<osg::Group> cam_group;
	osg::ref_ptr<osg::Group> skel_vis_group;
	//DepthMapPoly depth_poly;
	DepthMapSurfel depth_surf;
	osg::ref_ptr<osg::MatrixTransform> cam_pose_xform;
	std::map<int, std::vector< std::vector<float3> > > frame_surface_paths_3d;
private:
	void load_timestamps(std::map<int, double>& ts, std::string fn);
	void load_calibration(void);
	void load_frame(int frame_num);
	void segment_frames(void);
	void bilateral_filter_frames(void);
	bool cont_path(cv::Mat& d_img, cv::Point last, cv::Point current);
	void create_cam_geom(void);
	osg::Geode* create_cam_icon(osg::Vec3 vis_colour);
	float3 global_coord(int frame_num, int row, int col);
	std::string _dataset_path;
	std::string _cam_name;
	std::map<int, RGBD_Frame> frames;

	osg::Vec3 vis_colour;

	std::map<int, double> ts_depth_dev;
	std::map<int, double> ts_depth_global;
	std::map<int, double> ts_rgb_dev;
	std::map<int, double> ts_rgb_global;

	int d_rows;
	int d_cols;

	cv::Mat K_rgb;
	cv::Mat T_rgb;
};

#endif /* RGBDCAMERA_H_ */
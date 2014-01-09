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
#include <osg/LineWidth>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string/predicate.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/shared_ptr.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

class RGBD_Camera {
	public:
		RGBD_Camera(std::string dataset_path, std::string cam_name);
		virtual ~RGBD_Camera();
		int get_first_frame_num() const;
		int get_last_frame_num() const;
		int get_total_frame_num() const;
		int get_d_rows();
		int get_d_cols();
		osg::Vec3 get_vis_colour();
		cv::Mat get_K_rgb();
		float3x3 get_K_f3x3();
		cv::Mat get_inv_K_rgb();
		float3x3 get_inv_K_f3x3();
		cv::Mat get_T_rgb();
		float4x4 get_T_f4x4();
		osg::Matrix get_T();
		const cv::Mat* get_depth_map(int frame_num);
		cv::Mat* get_rgb_image(int frame_num);
		std::string get_cam_name();
		void get_surface_paths(int frame_num,
				std::vector<std::vector<cv::Point> >& surface_paths);
		void get_surface_paths_3d(int frame_num,
				std::vector<std::vector<float3> >& surface_paths_3d);
		const osg::Vec3& getVisColour() const;

		osg::ref_ptr<osg::Group> cam_group;
		osg::ref_ptr<osg::Group> skel_vis_group;
		//DepthMapPoly depth_poly;
		DepthMapSurfel depth_surf;
		osg::ref_ptr<osg::MatrixTransform> cam_pose_xform;
		std::map<int, std::vector<std::vector<float3> > > frame_surface_paths_3d;
	private:
		void load_timestamps(std::map<int, double>& ts, std::string fn);
		void load_calibration();
		void load_frame(int frame_num);
		void segment_frames();
		void bilateral_filter_frames();
		bool cont_path(cv::Mat& d_img, cv::Point last, cv::Point current);
		void create_cam_geom();
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
		cv::Mat inv_K_rgb;
		cv::Mat T_rgb;
		osg::Matrix T_osg;

		float3x3 K_d_f3x3;
		float3x3 inv_K_d_f3x3;
		float4x4 T_float4x4;
};

typedef std::vector<boost::shared_ptr<RGBD_Camera> > camVecT;
typedef std::vector<boost::shared_ptr<RGBD_Camera> >::iterator camVecIte;

#endif /* RGBDCAMERA_H_ */

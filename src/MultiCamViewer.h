/*
 * MultiCamViewer.h
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#ifndef MULTICAMVIEWER_H_
#define MULTICAMVIEWER_H_

#include "MiscUtils.h"
#include "RGBDCamera.h"
#include "DepthMapPoly.h"
#include "SurfelModel.h"
#include "SimpleTimer.h"
#include "RenderSkeletonization.h"
#include "SkeletonController.h"

#include <string>
#include <vector>
#include <fstream>

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
#include <osgUtil/SmoothingVisitor>
#include <osg/GraphicsContext>
#include <osg/BlendFunc>
#include <osg/LineWidth>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string/predicate.hpp"
#include "boost/lexical_cast.hpp"
#include "boost/shared_ptr.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

class MultiCamViewer: public osgGA::GUIEventHandler {
	public:
		MultiCamViewer(std::string path);
		virtual ~MultiCamViewer();
		int run_viewer(void);
	private:
		void setup_scene(void);
		void set_window_title(osgViewer::Viewer* viewer, std::string win_name,
				int x, int y);
		virtual bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);
		void update_dynamics(void);
		osgText::Text* create_text(const osg::Vec3& pos,
				const std::string& content, float size);
		osg::Camera* create_hud_camera(double left, double right, double bottom,
				double top);
		void save_image_freeview();

		int win_width;
		int win_height;

		int disp_frame_no;
		int begin_frame_no;
		int end_frame_no;
		bool paused;
		bool with_colour;
		float frame_period_s;
		double last_frame_tick_count;

		std::string _dataset_path;
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr;
		osg::ref_ptr<osg::Group> scene_root;
		osg::ref_ptr<osg::Image> rgb_render_interactive_view;
		osgViewer::Viewer viewer;

		//Switch to turn camera visualizations on or off:
		osg::ref_ptr<osg::Switch> cam_vis_switch;
		//Switch to turn skeleton on or off
		osg::ref_ptr<osg::Switch> skel_vis_switch;

		osg::ref_ptr<osg::Switch> skel_fitting_switch;

		osg::ref_ptr<osgText::Text> frame_num_text;
		float alpha;

		//RenderSkeletonization skel_renderer;
		SkeletonController skel_controller;

};

#endif /* MULTICAMVIEWER_H_ */

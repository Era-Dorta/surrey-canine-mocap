/*
 * SkeletonFitting.h
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#ifndef SKELETONCONTROLLER_H_
#define SKELETONCONTROLLER_H_

#include "Skeleton.h"
#include "RenderSkeletonization.h"
#include "Skeletonization3D.h"

#include <osgUtil/LineSegmentIntersector>
#include <osg/MatrixTransform>

#include <iostream>
using std::cout;
using std::endl;

enum Fitting_State {
	EMPTY, ADD_POINTS, MOVE_POINTS, POINTS_SET
};

class SkeletonController {
	public:
		SkeletonController();

		virtual ~SkeletonController();

		//Set root node for this class, it should be call after creation
		void set_data(osg::ref_ptr<osg::Switch> skel_fitting_switch,
				std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr,
				osg::ref_ptr<osg::Switch> skel_vis_switch);

		Fitting_State getState() const;
		void setState(Fitting_State state);

		//Handle mouse events, to set up fitting points
		bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);

		void load_skeleton_from_file(std::string file_name);
		void save_skeleton_to_file(std::string file_name);

		void update_dynamics(int disp_frame_no);
	private:
		void set_skeleton_point();
		void reset_state();
		void update_state();
		void draw_complete_skeleton();

		//Class that creates a skeleton from a given set of frames
		Skeletonization3D skeletonized3D;

		//Class that renders all skeleton related objects
		RenderSkeletonization skel_renderer;

		Fitting_State state;
		bool point_selected;
		osg::ref_ptr<osg::MatrixTransform> selected_point;
		int selected_point_index;
		Skeleton skeleton;
		int current_frame;
};

#endif /* SKELETONCONTROLLER_H_ */

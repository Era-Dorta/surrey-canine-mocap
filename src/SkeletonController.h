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
#include "SkeletonFitting.h"

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
		void set_data(std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr,
				osg::ref_ptr<osg::Group> render_skel_group);

		Fitting_State getState() const;
		void setState(Fitting_State state);

		//Handle mouse events, to set up fitting points
		bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);

		void update_dynamics(int disp_frame_no);
	private:
		void reset_state();
		void update_state();
		void draw_edit_text();

		void load_skeleton_from_file(std::string file_name);
		void save_skeleton_to_file(std::string file_name);

		//Handle mouse events, to set up fitting points
		bool handle_mouse_events(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);

		//Handle mouse events, to set up fitting points
		bool handle_keyboard_events(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);

		//Class that creates a skeleton from a given set of frames
		Skeletonization3D skeletonized3D;

		//Class that renders all skeleton related objects
		RenderSkeletonization skel_renderer;

		//Class with methods to modify(and create???) a skeleton to fit into a
		//cloud of points that represent a skeleton.
		SkeletonFitting skel_fitter;

		Fitting_State state;
		bool is_point_selected;
		int selected_point_index;
		osg::ref_ptr<osg::MatrixTransform> selected_point_color;
		Skeleton skeleton;
		int current_frame;
		int last_mouse_pos_x;
		int last_mouse_pos_y;
		bool move_on_z;
		bool translate_root;
		bool change_all_frames;
		bool transforming_skeleton;
		osg::Vec4 unselected_color;
		osg::Vec4 selected_color;
		unsigned int inter_number;
};

#endif /* SKELETONCONTROLLER_H_ */

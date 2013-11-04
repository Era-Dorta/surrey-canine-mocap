/*
 * SkeletonFitting.h
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#ifndef SKELETONFITCONTROLLER_H_
#define SKELETONFITCONTROLLER_H_

#include "SkeletonFitting.h"

#include <osgGA/GUIEventHandler>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

#include <iostream>
using std::cout;
using std::endl;

enum Fitting_State {
	EMPTY, ADD_POINTS, MOVE_POINTS, POINTS_SET
};

class SkeletonFitController: public osgGA::GUIEventHandler {
	public:
		SkeletonFitController();

		virtual ~SkeletonFitController();

		//Set root node for this class, it should be call after creation
		void set_data(osg::ref_ptr<osg::Switch> root_node);

		Fitting_State getState() const;
		void setState(Fitting_State state);

		//Handle mouse events, to set up fitting points
		virtual bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);

		void load_skeleton_from_file(std::string file_name);
		void save_skeleton_to_file(std::string file_name);

		//TODO Move all drawing related code to render skeletonization
		//or create another render class, but not here, this is a controller
		void update_dynamics(int disp_frame_no);
	private:
		void set_skeleton_point();
		void change_colour_when_selected();
		void reset_state();
		void update_state();
		void draw_bone(osg::Vec3& bone_start, osg::Vec3& bone_end);
		void draw_complete_skeleton();
		void draw_joints();
		void clear_scene();

		osg::ref_ptr<osg::MatrixTransform> createSelectionBox();

		//osg::ref_ptr<osg::MatrixTransform> _selectionBox;
		osg::ref_ptr<osg::Switch> skel_fitting_switch;
		Fitting_State state;
		bool point_selected;
		osg::ref_ptr<osg::MatrixTransform> selected_point;
		int selected_point_index;
		osg::Vec4 joint_colour;
		osg::Vec4 bone_colour;
		osg::Vec4 selection_colour;
		SkeletonFitting skel_fitting;
};

#endif /* SKELETONFITCONTROLLER_H_ */

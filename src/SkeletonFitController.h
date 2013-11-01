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
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

#include <iostream>
using std::cout;
using std::endl;

enum Fitting_State { EMPTY, ADD_POINTS, MOVE_POINTS, POINTS_SET };

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

	private:
		void set_skeleton_point();

		osg::ref_ptr<osg::MatrixTransform> createSelectionBox();

		//osg::ref_ptr<osg::MatrixTransform> _selectionBox;
		osg::ref_ptr<osg::Switch> skel_fitting_switch;
		Fitting_State state;
		bool point_selected;
		osg::ref_ptr<osg::MatrixTransform> selected_point;
		int selected_point_index;
		SkeletonFitting skel_fitting;
};

#endif /* SKELETONFITCONTROLLER_H_ */

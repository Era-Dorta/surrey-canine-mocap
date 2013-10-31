/*
 * SkeletonFitting.h
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#ifndef SKELETONFITTING_H_
#define SKELETONFITTING_H_

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

class SkeletonFitting: public osgGA::GUIEventHandler {
	public:
		SkeletonFitting();

		virtual ~SkeletonFitting();

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
		unsigned int points_added;
};

#endif /* SKELETONFITTING_H_ */

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

class SkeletonFitting: public osgGA::GUIEventHandler {
	public:
		SkeletonFitting();
		virtual ~SkeletonFitting();
		void set_data(osg::ref_ptr<osg::Switch> root_node);
		virtual bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);
	private:
		void set_skeleton_point();
		osg::Node* getOrCreateSelectionBox();

		osg::ref_ptr<osg::MatrixTransform> _selectionBox;
		osg::ref_ptr<osg::Switch> skel_fitting_switch;
};

#endif /* SKELETONFITTING_H_ */

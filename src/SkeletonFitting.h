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
		virtual bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);

		osg::Node* getOrCreateSelectionBox();
	private:
		void set_skeleton_point();

		osg::ref_ptr<osg::MatrixTransform> _selectionBox;
};

#endif /* SKELETONFITTING_H_ */

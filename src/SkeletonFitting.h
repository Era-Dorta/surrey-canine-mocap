/*
 * SkeletonFitting.h
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#ifndef SKELETONFITTING_H_
#define SKELETONFITTING_H_

#include <osgGA/GUIEventHandler>

#include <iostream>
using std::cout;
using std::endl;

class SkeletonFitting : public osgGA::GUIEventHandler {
	public:
		SkeletonFitting();
		virtual ~SkeletonFitting();
		virtual bool handle(const osgGA::GUIEventAdapter& ea,
				osgGA::GUIActionAdapter& aa);
	private:
		void set_skeleton_point();
};

#endif /* SKELETONFITTING_H_ */

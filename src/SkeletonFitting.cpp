/*
 * SkeletonFitting.cpp
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"

SkeletonFitting::SkeletonFitting() {
	// TODO Auto-generated constructor stub

}

SkeletonFitting::~SkeletonFitting() {
	// TODO Auto-generated destructor stub
}

bool SkeletonFitting::handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa){
	if ( ea.getEventType()== osgGA::GUIEventAdapter::RELEASE &&
			ea.getButton()== osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON
			&&
			(ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL)
			){
		cout << "handling bymyself" << endl;
	}
	return false;
}

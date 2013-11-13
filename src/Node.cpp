/*
 * Node.cpp
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */
#include "Node.h"

NODE::NODE() {
	noofchildren = 0;
	parent = NULL;
	froset = new osg::Vec3Array;
	freuler = new osg::Vec3Array;
	DOFs = 0;
	noofchannels = 0;
}

NODE::~NODE() {
}


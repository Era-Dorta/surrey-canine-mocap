/*
 * MocapHeader.cpp
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */

#include "MocapHeader.h"

MocapHeader::MocapHeader() {
	noofsegments = 0;
	noofframes = 0;
	datarate = 0;
	frametime = 0.0;
	euler = new osg::Vec3Array;
	euler->resize(3);
	callib = 1.0;
	degrees = true;
	scalefactor = 1.0;
	currentframe = 0;
}

MocapHeader::~MocapHeader() {

}


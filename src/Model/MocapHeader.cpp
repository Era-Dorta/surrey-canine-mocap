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
	euler = new osg::Vec3Array(3);

	//This matrix indicates the axis in which the rotations take place
	//files are going to be read and written as x, y, z rotations
	euler->at(0).set(1, 0, 0);
	euler->at(1).set(0, 1, 0);
	euler->at(2).set(0, 0, 1);

	callib = 0.6f;
	inv_callib = 1.0 / callib;
	degrees = false;
	scalefactor = 1.0;
	currentframe = 0;
}

MocapHeader::~MocapHeader() {

}


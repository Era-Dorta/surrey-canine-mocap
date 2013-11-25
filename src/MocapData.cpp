/*
 * MocapData.cpp
 *
 *  Created on: 11 Nov 2013
 *      Author: m04701
 */

#include "MocapData.h"

MocapData::MocapData() :
			xpos(0), ypos(0), zpos(0) {
}

const std::vector<Node*>& MocapData::get_node_list() const {
	return nodelist;
}

void MocapData::reset_state() {
	nodelist.clear();
	root.reset();

	header.callib = 1.0f;
	header.scalefactor = 1.0f;
	header.noofsegments = 0;
	header.noofframes = 0;
	header.datarate = 0;

	xpos = 1;
	ypos = 2;
	zpos = 0;

	//This matrix indicates the axes in which the rotations take place
	//files are going to be read and written as x, y, z rotations
	header.euler->at(0).set(1, 0, 0);
	header.euler->at(1).set(0, 1, 0);
	header.euler->at(2).set(0, 0, 1);

	//This calibration to make every model smaller
	header.callib = 0.3f;
	header.inv_callib = 1.0 / header.callib;
	//We convert all values to radians
	header.degrees = false;
	header.scalefactor = 1.0f;
}

MocapData::~MocapData() {

}

Node* MocapData::get_root() {
	return root.get();
}

const char* MocapData::get_error() {
	return error;
}

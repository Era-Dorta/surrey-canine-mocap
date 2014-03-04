/*
 * MocapData.cpp
 *
 *  Created on: 11 Nov 2013
 *      Author: m04701
 */

#include "MocapData.h"

MocapData::MocapData() {
}

const std::vector<Node*>& MocapData::get_node_list() const {
	return nodelist;
}

Node* MocapData::get_node(unsigned int index) const {
	return nodelist.at(index);
}

void MocapData::reset_state() {
	nodelist.clear();
	root.reset();
	header = MocapHeader();
}

MocapData::~MocapData() {

}

Node* MocapData::get_root() {
	return root.get();
}

const char* MocapData::get_error() {
	return error;
}

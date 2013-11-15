/*
 * MocapData.cpp
 *
 *  Created on: 11 Nov 2013
 *      Author: m04701
 */

#include "MocapData.h"

MocapData::MocapData() :
			root(0), nodelist(0), header(0), xpos(0), ypos(0), zpos(0) {
}

MocapData::MocapData(MocapHeader *header) :
			root(0), nodelist(0), header(header), xpos(0), ypos(0), zpos(0) {
}

void MocapData::free_node_memory(struct Node* to_delete) {
	free(to_delete);
}

const std::vector<Node*>& MocapData::getNodelist() const {
	return nodelist;
}

void MocapData::reset_state() {
	delete_recursive(root);
	root = NULL;
	nodelist.clear();
}

void MocapData::delete_recursive(struct Node* to_delete) {
	if (to_delete == NULL) {
		return;
	}
	for (int i = 0; i < to_delete->noofchildren; i++) {
		delete_recursive(to_delete->children[i]);
	}
	free_node_memory(to_delete);
}

MocapData::~MocapData() {
	delete_recursive(root);
}

void MocapData::SetHeader(MocapHeader *header) {
	this->header = header;
}

Node* MocapData::GetRootNode() {
	return root;
}

const char* MocapData::GetError() {
	return error;
}

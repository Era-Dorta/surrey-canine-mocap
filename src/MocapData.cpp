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

MocapData::MocapData(MOCAPHEADER *header) :
			root(0), nodelist(0), header(header), xpos(0), ypos(0), zpos(0) {
}

void MocapData::free_node_memory(struct NODE* to_delete) {
	for (long i = 0; i < header->noofframes; i++) {
		free(to_delete->froset[i]);
		free(to_delete->freuler[i]);
	}
	free(to_delete->froset);
	free(to_delete->freuler);
	free(to_delete->scale);

	free(to_delete->name);

	free(to_delete);
}

void MocapData::delete_recursive(struct NODE* to_delete) {
	cout << "delete recursive" << endl;
	if (to_delete == NULL) {
		return;
	}
	for (int i = 0; i < to_delete->noofchildren; i++) {
		delete_recursive(to_delete->children[i]);
	}
	if(to_delete->noofchildren > 0){
		free(to_delete->children);
	}
	free_node_memory(to_delete);
}

MocapData::~MocapData() {
	delete_recursive(root);
	free(nodelist);
}

void MocapData::SetHeader(MOCAPHEADER *header) {
	this->header = header;
}

NODE* MocapData::GetRootNode() {
	return root;
}

NODE** MocapData::GetNodeList() {
	return nodelist;
}

const char* MocapData::GetError() {
	return error;
}

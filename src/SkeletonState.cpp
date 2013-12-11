/*
 * SkeletonState.cpp
 *
 *  Created on: 11 Dec 2013
 *      Author: m04701
 */

#include "SkeletonState.h"

SkeletonState::SkeletonState() {
	children_offset = new osg::Vec3Array;
}

SkeletonState::~SkeletonState() {
	// TODO Auto-generated destructor stub
}

void SkeletonState::save_state(boost::shared_ptr<Skeleton> skeleton,
		int frame_num, unsigned int node_index) {
	children_offset->clear();
	Node* node = skeleton->get_node(node_index);
	node_rotation = node->quat_arr.at(frame_num);
	node_offset = node->offset;
	node_length = node->length;

	children_offset->reserve(node->get_num_children());
	for (unsigned int i = 0; i < node->get_num_children(); i++) {
		children_offset->push_back(node->children[i]->offset);
	}

	if (node->parent) {
		node = node->parent;
		parent_rotation = node->quat_arr.at(frame_num);
		parent_offset = node->offset;
		parent_length = node->length;
	}
}

void SkeletonState::restore_state(boost::shared_ptr<Skeleton> skeleton,
		int frame_num, unsigned int node_index) {
	Node* node = skeleton->get_node(node_index);
	node->quat_arr.at(frame_num) = node_rotation;
	node->offset = node_offset;
	node->length = node_length;

	for (unsigned int i = 0; i < node->get_num_children(); i++) {
		node->children[i]->offset = children_offset->at(i);
	}

	if (node->parent) {
		node = node->parent;
		node->quat_arr.at(frame_num) = parent_rotation;
		node->offset = parent_offset;
		node->length = parent_length;
	}
}

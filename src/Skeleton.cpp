/*
 * SkeletonFitting.cpp
 *
 *  Created on: 1 Nov 2013
 *      Author: m04701
 */

#include "Skeleton.h"

Skeleton::Skeleton() :
			skel_loaded(false) {
	SetHeader(&mocap_header);
	rotate_scale_factor = 0.2;
	translate_scale_factor = 0.002;
}

Skeleton::~Skeleton() {
}

void Skeleton::rotate_joint(unsigned int index, osg::Vec3& angle) {
	angle *= rotate_scale_factor;
	nodelist[index]->freuler->at(mocap_header.currentframe) += angle;
}

void Skeleton::rotate_every_frame(osg::Vec3& angle) {
	angle *= rotate_scale_factor;
	osg::Vec3Array::iterator i;
	for (i = root->freuler->begin(); i != root->freuler->end(); ++i) {
		(*i) += angle;
	}
}

void Skeleton::translate_root(osg::Vec3& translation) {
	translation *= translate_scale_factor;
	root->froset->at(mocap_header.currentframe) += translation;
}

void Skeleton::translate_every_frame(osg::Vec3& translation) {
	translation *= translate_scale_factor;
	root->offset += translation;
}

void Skeleton::save_to_file(std::string file_name) {
	ExportData(file_name.c_str());
}

void Skeleton::load_from_file(std::string file_name) {
	ImportData(file_name.c_str());
	skel_loaded = true;
}

unsigned int Skeleton::get_num_bones() {
	return mocap_header.noofsegments;
}

void Skeleton::set_current_frame(int frame_no) {
	mocap_header.currentframe = frame_no;

	//Frame beyond vector
	if (frame_no >= (long) mocap_header.noofframes) {
		for (int i = mocap_header.noofframes; i <= frame_no; i++) {
			for (NodeIte j = nodelist.begin(); j != nodelist.end(); ++j) {
				(*j)->freuler->push_back((*j)->freuler->back());
				(*j)->froset->push_back((*j)->freuler->back());
			}
		}
		mocap_header.noofframes = frame_no + 1;
	}
}

Node* Skeleton::get_root() {
	return GetRootNode();
}

MocapHeader& Skeleton::get_header() {
	return mocap_header;
}

void Skeleton::reset_state() {
	set_current_frame(mocap_header.currentframe);
}

int Skeleton::get_node(osg::ref_ptr<osg::MatrixTransform> node_transform) {
	for (unsigned int i = 0; i < nodelist.size(); i++) {
		if (nodelist[i]->osg_node.get() == node_transform.get()) {
			return i;
		}
	}
	return -1;
}

bool Skeleton::isSkelLoaded() const {
	return skel_loaded;
}

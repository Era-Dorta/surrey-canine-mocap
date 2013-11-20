/*
 * SkeletonFitting.cpp
 *
 *  Created on: 1 Nov 2013
 *      Author: m04701
 */

#include "Skeleton.h"
#include "DebugUtil.h"
Skeleton::Skeleton() :
			skel_loaded(false) {
	rotate_scale_factor = 0.02;
	translate_scale_factor = 0.002;
	//TODO This colors are defined here and in Node, should only be in one place
	joint_color = osg::Vec4(0.5f, 0.5f, 0.5f, 1.0); //Grey
	joint_second_color = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0); //White
	bone_color = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0); //Blue
	bone_second_color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0); //Red
}

Skeleton::~Skeleton() {
}

void Skeleton::rotate_joint(unsigned int index, osg::Vec3& angle) {
	angle *= rotate_scale_factor;
	nodelist[index]->freuler->at(header.currentframe) += angle;
}

void Skeleton::rotate_root_every_frame(osg::Vec3& angle) {
	angle *= rotate_scale_factor;
	osg::Vec3Array::iterator i;
	for (i = root->freuler->begin(); i != root->freuler->end(); ++i) {
		(*i) += angle;
	}
}

void Skeleton::translate_joint(unsigned int index, osg::Vec3& translation) {
	translation *= translate_scale_factor;
	nodelist[index]->froset->at(header.currentframe) += translation;
}

void Skeleton::translate_every_frame(unsigned int index,
		osg::Vec3& translation) {
	translation *= translate_scale_factor;
	nodelist[index]->length += translation;
	for (unsigned int i = 0; i < nodelist[index]->noofchildren(); i++) {
		nodelist[index]->children[i]->offset += translation;
	}
}

void Skeleton::save_to_file(std::string file_name) {
	export_data(file_name.c_str());
}

void Skeleton::load_from_file(std::string file_name) {
	reset_state();
	import_data(file_name.c_str());
	skel_loaded = true;
}

unsigned int Skeleton::get_num_bones() {
	return header.noofsegments;
}

void Skeleton::set_current_frame(int frame_no) {
	header.currentframe = frame_no;

	//Frame beyond vector
	if (frame_no >= (long) header.noofframes) {
		for (int i = header.noofframes; i <= frame_no; i++) {
			for (NodeIte j = nodelist.begin(); j != nodelist.end(); ++j) {
				(*j)->freuler->push_back((*j)->freuler->back());
				(*j)->froset->push_back((*j)->freuler->back());
			}
		}
		header.noofframes = frame_no + 1;
	}
}

MocapHeader& Skeleton::get_header() {
	return header;
}

void Skeleton::toggle_color(int index) {
	if (nodelist[index]->joint_color != joint_color) {
		nodelist[index]->joint_color = joint_color;
		nodelist[index]->bone_color = bone_color;
	} else {
		nodelist[index]->joint_color = joint_second_color;
		nodelist[index]->bone_color = bone_second_color;
	}
}

int Skeleton::get_node_index(osg::MatrixTransform* node_transform) {
	for (unsigned int i = 0; i < nodelist.size(); i++) {
		if (nodelist[i]->osg_node == node_transform) {
			return i;
		}
	}
	return -1;
}

bool Skeleton::isSkelLoaded() const {
	return skel_loaded;
}

void Skeleton::reset_state() {
	BVHFormat::reset_state();
	skel_loaded = false;
}

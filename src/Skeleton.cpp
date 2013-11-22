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
}

Skeleton::~Skeleton() {
}

void Skeleton::rotate_joint(unsigned int index, osg::Vec3& angle) {
	angle *= rotate_scale_factor;
	osg::Quat new_rot(angle[0], header.euler->at(0), angle[1],
			header.euler->at(1), angle[2], header.euler->at(2));
	nodelist[index]->quat_arr.at(header.currentframe) =
			nodelist[index]->quat_arr.at(header.currentframe) * new_rot;
}

void Skeleton::rotate_root_all_frames(osg::Vec3& angle) {
	angle *= rotate_scale_factor;
	osg::Quat new_rot(angle[0], header.euler->at(0), angle[1],
			header.euler->at(1), angle[2], header.euler->at(2));
	std::vector<osg::Quat>::iterator i;
	for (i = root->quat_arr.begin(); i != root->quat_arr.end(); ++i) {
		(*i) = (*i) * new_rot;
	}
}

void Skeleton::translate_root(osg::Vec3& translation) {
	translation *= translate_scale_factor;
	root->froset->at(header.currentframe) += translation;
}

void Skeleton::translate_root_all_frames(osg::Vec3& translation) {
	translation *= translate_scale_factor;
	root->offset += translation;
}

void Skeleton::change_bone_length(unsigned int index, osg::Vec3& translation) {
	translation *= translate_scale_factor;
	nodelist[index]->length += translation;
	for (unsigned int i = 0; i < nodelist[index]->get_num_children(); i++) {
		nodelist[index]->children[i]->froset->at(header.currentframe) +=
				translation;
	}
}

void Skeleton::change_bone_length_all_frames(unsigned int index,
		osg::Vec3& translation) {
	translation *= translate_scale_factor;
	nodelist[index]->length += translation;
	for (unsigned int i = 0; i < nodelist[index]->get_num_children(); i++) {
		nodelist[index]->children[i]->offset += translation;
	}
}

void Skeleton::save_to_file(std::string file_name) {
	NodeIte i;
	for (i = nodelist.begin(); i != nodelist.end(); ++i) {
		(*i)->update_euler_angles();
	}
	export_data(file_name.c_str());
}

void Skeleton::load_from_file(std::string file_name) {
	reset_state();
	import_data(file_name.c_str());
	NodeIte i;
	for (i = nodelist.begin(); i != nodelist.end(); ++i) {
		(*i)->calculate_quats(header.euler);
	}
	skel_loaded = true;
}

unsigned int Skeleton::get_num_bones() {
	return header.noofsegments;
}

void Skeleton::set_current_frame(int frame_no) {
	header.currentframe = frame_no;

	//Frame beyond vector
	if (frame_no >= (long) header.noofframes) {
		header.noofframes = frame_no + 1;
		for (NodeIte j = nodelist.begin(); j != nodelist.end(); ++j) {
			(*j)->resize_frame_no(header.noofframes);
		}
	}
}

MocapHeader& Skeleton::get_header() {
	return header;
}

void Skeleton::toggle_color(int index) {
	nodelist[index]->toggle_color();
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

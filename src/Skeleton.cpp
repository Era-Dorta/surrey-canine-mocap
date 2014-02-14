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
}

Skeleton::~Skeleton() {
}

void Skeleton::rotate_joint(unsigned int index, const osg::Vec3& angle) {
	osg::Quat new_rot(angle[0], header.euler->at(0), angle[1],
			header.euler->at(1), angle[2], header.euler->at(2));
	nodelist[index]->quat_arr.at(header.currentframe) =
			nodelist[index]->quat_arr.at(header.currentframe) * new_rot;
}

void Skeleton::rotate_root_all_frames(const osg::Vec3& angle) {
	osg::Quat new_rot(angle[0], header.euler->at(0), angle[1],
			header.euler->at(1), angle[2], header.euler->at(2));
	std::vector<osg::Quat>::iterator i;
	for (i = root->quat_arr.begin(); i != root->quat_arr.end(); ++i) {
		(*i) = (*i) * new_rot;
	}
}

void Skeleton::translate_root(const osg::Vec3& translation) {
	root->froset->at(header.currentframe) += translation;
}

void Skeleton::translate_root_all_frames(const osg::Vec3& translation) {
	root->offset += translation;
}

void Skeleton::change_bone_length(unsigned int index,
		const osg::Vec3& translation) {
	nodelist[index]->length += translation;
	for (unsigned int i = 0; i < nodelist[index]->get_num_children(); i++) {
		nodelist[index]->children[i]->froset->at(header.currentframe) +=
				translation;
	}
}

void Skeleton::change_bone_length_all_frames(unsigned int index,
		const osg::Vec3& translation) {

	//Bone length is easier along global axis
	osg::Quat inv_glob_rot = nodelist[index]->get_inv_global_rot(
			header.currentframe);

	osg::Vec3 trans_local_coor = inv_glob_rot * translation;
	//Inverse bone rotation and translate in world coordinates
	nodelist[index]->length += trans_local_coor;

	//Translate also all brothers to maintain skeleton connectivity
	for (unsigned int i = 0; i < nodelist[index]->get_num_children(); i++) {
		nodelist[index]->children[i]->offset += trans_local_coor;
	}
}

void Skeleton::rotate_two_bones_keep_end_pos(unsigned int index, float angle) {
	unsigned int prev_index = index - 1;

	if (prev_index < 0) {
		return;
	}

	Node* first_bone = nodelist.at(prev_index);
	Node* second_bone = nodelist.at(index);

	osg::Matrix m;
	second_bone->get_parent_to_bone_end_matrix(header.currentframe, m);

	//Calculate the end position relative to the first bone
	osg::Vec3 dir_vec = osg::Vec3() * m;

	dir_vec.normalize();

	//The rotation that maintains the end position is along the
	//calculated vector
	osg::Quat new_rot(angle, dir_vec);

	//Rotate first bone along the previous vector
	first_bone->quat_arr.at(header.currentframe) *= new_rot;
	return;
}

void Skeleton::rotate_two_bones_keep_end_pos_aim(unsigned int index,
		float angle) {
	unsigned int prev_index = index - 1;

	if (prev_index < 0) {
		return;
	}

	Node* first_bone = nodelist.at(prev_index);
	Node* second_bone = nodelist.at(index);

	osg::Matrix m;

	//TODO Axes calculation could be done only once and saved for next
	//bone rotations
	second_bone->get_parent_to_bone_end_matrix(header.currentframe, m);

	//Calculate the end position relative to the first bone
	osg::Vec3 dir_vec = osg::Vec3() * m;

	dir_vec.normalize();

	//The rotation that maintains the end position is along the
	//calculated vector
	osg::Quat new_rot(angle, dir_vec);

	//Rotate first bone along the previous vector
	first_bone->quat_arr.at(header.currentframe) *= new_rot;

	//The rotation vector has to be in child coordinate system
	osg::Vec3 new_vec = dir_vec * osg::Matrix::inverse(m);

	//Rotate all children in the opposite direction so they remain
	//in the same position
	new_rot = osg::Quat(-angle, new_vec);
	for (unsigned int i = 0; i < second_bone->get_num_children(); i++) {
		second_bone->children.at(i)->quat_arr.at(header.currentframe) *=
				new_rot;
	}

	return;
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

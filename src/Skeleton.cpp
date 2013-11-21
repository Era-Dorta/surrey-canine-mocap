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
	/*cout << "angle before " << angle << endl;
	 translate_coord_to_global(index, angle);
	 cout << "angle after " << angle << endl << endl;
	 nodelist[index]->freuler->at(header.currentframe) += angle;*/
	osg::Matrix new_rot = osg::Matrix::rotate(angle[0], header.euler->at(0),
			angle[1], header.euler->at(1), angle[2], header.euler->at(2));
	nodelist[index]->freuler_m.at(header.currentframe) =
			nodelist[index]->freuler_m.at(header.currentframe) * new_rot;
}

void Skeleton::rotate_root_every_frame(osg::Vec3& angle) {
	angle *= rotate_scale_factor;
	translate_coord_to_global(0, angle);
	osg::Vec3Array::iterator i;
	for (i = root->freuler->begin(); i != root->freuler->end(); ++i) {
		(*i) += angle;
	}
}

void Skeleton::translate_joint(unsigned int index, osg::Vec3& translation) {
	translation *= translate_scale_factor;
	translate_coord_to_global(index, translation);
	nodelist[index]->froset->at(header.currentframe) += translation;
}

void Skeleton::translate_every_frame(unsigned int index,
		osg::Vec3& translation) {
	translation *= translate_scale_factor;
	translate_coord_to_global(index, translation);
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
	NodeIte i;
	for (i = nodelist.begin(); i != nodelist.end(); ++i) {
		(*i)->calculate_matrices(header.euler);
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

osg::Matrixd Skeleton::get_joint_transformation(int index) {
	osg::Matrixd result, aux;
	result.makeIdentity();
	Node* current;
	current = nodelist[index];

	while (current != NULL) {
		aux = osg::Matrix::rotate(current->freuler->at(header.currentframe)[0],
				header.euler->at(0),
				current->freuler->at(header.currentframe)[1],
				header.euler->at(1),
				current->freuler->at(header.currentframe)[2],
				header.euler->at(2))

				* osg::Matrix::translate(
						current->offset
								+ current->froset->at(header.currentframe));
		result = aux * result;
		current = current->parent;
	}
	return result;
}

void Skeleton::translate_coord_to_global(int index, osg::Vec3& v) {
	/*const osg::Matrix joint_matrix = get_joint_transformation(index);
	 cout << "joint_matix is " << joint_matrix << endl;
	 osg::Matrix joint_matrix_inv;
	 osg::Matrix joint_matrix_inv2;


	 joint_matrix_inv = osg::Matrix::inverse(joint_matrix);
	 cout << "joint_matix inv is " << joint_matrix_inv << endl;
	 cout << "joint_matix after" << joint_matrix << endl;*/

	osg::Matrix rot_y, inv_rot_y, rot_z, inv_rot_z;

	rot_y = osg::Matrix::rotate(
			nodelist[index]->freuler->at(header.currentframe)[1],
			header.euler->at(1));
	inv_rot_y = osg::Matrix::inverse(rot_y);

	rot_z = osg::Matrix::rotate(
			nodelist[index]->freuler->at(header.currentframe)[2],
			header.euler->at(2));
	inv_rot_z = osg::Matrix::inverse(rot_z);

	if (v[0] != 0.0) {
		v = inv_rot_y * inv_rot_z * v;
		cout << "x rotation" << endl;
		return;
	}

	if (v[1] != 0.0) {
		v = inv_rot_z * v;
		cout << "y rotation" << endl;
		return;
	}
}

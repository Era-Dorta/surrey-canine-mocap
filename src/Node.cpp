/*
 * Node.cpp
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */
#include "Node.h"

const osg::Vec4 Node::joint_color(0.5f, 0.5f, 0.5f, 1.0); //Grey
const osg::Vec4 Node::joint_second_color(1.0f, 1.0f, 1.0f, 1.0); //White
const osg::Vec4 Node::bone_color(0.0f, 0.0f, 1.0f, 1.0); //Blue
const osg::Vec4 Node::bone_second_color(1.0f, 0.0f, 0.0f, 1.0); //Red

const float Node::bone_radius = 0.005;
const float Node::joint_radius = 0.01;

Node::Node() {
	parent = NULL;
	froset = new osg::Vec3Array;
	freuler = new osg::Vec3Array;
	DOFs = 0;
	noofchannels = 0;
	osg_node = NULL;
	n_joint_color = joint_color;
	n_bone_color = bone_color;
}

Node::~Node() {
}

void Node::setup_offset(float x, float y, float z) {
	offset[0] = x;
	offset[1] = y;
	offset[2] = z;
}

void Node::increase_no_children() {
	children.push_back(NodePtr(new Node));
	children.back()->parent = this;
}

void Node::resize_frame_no(long frames) {
	//If the vectors are not empty then resize but fill with the value of the
	//last element
	if (freuler->size() > 0) {
		freuler->resize(frames, freuler->back());
		froset->resize(frames, froset->back());
		quat_arr.resize(frames, quat_arr.back());
	} else {
		freuler->resize(frames);
		froset->resize(frames);
		quat_arr.resize(frames);
	}
}

unsigned int Node::get_num_children() {
	return children.size();
}

Node* Node::get_last_child() {
	return children.back().get();
}

void Node::calculate_quats(osg::ref_ptr<osg::Vec3Array> axis) {
	//To build the quaternion we use an axis and a rotation around that axis
	//from the bvh file. Since OSG uses premultiplication the order of the
	//rotations is build backwards
	for (unsigned int i = 0; i < freuler->size(); i++) {
		quat_arr.at(i) = osg::Quat(freuler->at(i)[2], axis->at(2),
				freuler->at(i)[1], axis->at(1), freuler->at(i)[0], axis->at(0));
	}
}

void Node::update_euler_angles() {
	for (unsigned int i = 0; i < quat_arr.size(); i++) {
		quat_to_euler(quat_arr.at(i), freuler->at(i));
	}
}

osg::Quat Node::get_global_rot(int frame_num) {
	osg::Quat res;
	Node * prev_node = this;

	do {
		res = prev_node->quat_arr.at(frame_num) * res;
		prev_node = prev_node->parent;
	} while (prev_node != NULL);

	return res;
}

osg::Quat Node::get_inv_global_rot(int frame_num) {
	return get_global_rot(frame_num).inverse();
}

osg::Vec3 Node::get_global_pos(int frame_num) {
	Node * prev_node = this;
	osg::Matrix aux;
	do {
		aux = aux * osg::Matrix::rotate(prev_node->quat_arr.at(frame_num))
				* osg::Matrix::translate(
						prev_node->offset + prev_node->froset->at(frame_num));
		prev_node = prev_node->parent;
	} while (prev_node != NULL);

	return osg::Vec3() * aux;
}

osg::Vec3 Node::get_end_bone_global_pos(int frame_num) {
	Node * prev_node = this;
	osg::Matrix aux;
	do {
		aux = aux * osg::Matrix::rotate(prev_node->quat_arr.at(frame_num))
				* osg::Matrix::translate(
						prev_node->offset + prev_node->froset->at(frame_num));
		prev_node = prev_node->parent;
	} while (prev_node != NULL);
	aux = osg::Matrix::translate(length) * aux;

	return osg::Vec3() * aux;
}

void Node::quat_to_euler(osg::Quat& q, osg::Vec3& euler) {
	//To calculate back the euler angles from a quaternion we use this formulas
	//but since we want the postmultiplication order we use the conjugate of the
	//quaternion, also there has to be a change in the sign of the angles
	//http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	osg::Quat q_c = q.conj();
	euler[0] = -std::atan2(2.0 * (q_c.y() * q_c.z() + q_c.w() * q_c.x()),
			q_c.w() * q_c.w() - q_c.x() * q_c.x() - q_c.y() * q_c.y()
					+ q_c.z() * q_c.z());
	euler[1] = -std::asin(-2.0 * (q_c.x() * q_c.z() - q_c.w() * q_c.y()));
	euler[2] = -std::atan2(2.0 * (q_c.x() * q_c.y() + q_c.w() * q_c.z()),
			q_c.w() * q_c.w() + q_c.x() * q_c.x() - q_c.y() * q_c.y()
					- q_c.z() * q_c.z());
}

void Node::toggle_color() {
	if (n_joint_color != joint_color) {
		n_joint_color = joint_color;
		n_bone_color = bone_color;
	} else {
		n_joint_color = joint_second_color;
		n_bone_color = bone_second_color;
	}
}

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
	for (unsigned int i = 0; i < freuler->size(); i++) {
		quat_arr.at(i) = osg::Quat(freuler->at(i)[0], axis->at(0),
				freuler->at(i)[1], axis->at(1), freuler->at(i)[2], axis->at(2));
	}
}

void Node::update_euler_angles() {
	for (unsigned int i = 0; i < freuler->size(); i++) {
		quat_to_euler(quat_arr.at(i), freuler->at(i));
	}
}

void Node::quat_to_euler(osg::Quat& q, osg::Vec3& euler) {
	//Quat formula to euler from http://glm.g-truc.net
	euler[0] = std::atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
			q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
	euler[1] = std::asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
	euler[2] = std::atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
			q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
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

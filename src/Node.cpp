/*
 * Node.cpp
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */
#include "Node.h"

Node::Node() {
	parent = NULL;
	froset = new osg::Vec3Array;
	freuler = new osg::Vec3Array;
	DOFs = 0;
	noofchannels = 0;
	joint_color = osg::Vec4(0.5f, 0.5f, 0.5f, 1.0);
	bone_color = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0);
	osg_node = NULL;
}

Node::~Node() {
}

void Node::setup_children(int new_no_children) {
	children.resize(new_no_children);
}

void Node::setup_offset(float x, float y, float z) {
	offset[0] = x;
	offset[1] = y;
	offset[2] = z;
}

void Node::setup_euler(float r1, float r2, float r3) {
	euler[0] = r1;
	euler[1] = r2;
	euler[2] = r3;
}

void Node::setup_color(float r, float g, float b) {
	joint_color[0] = r;
	joint_color[1] = g;
	joint_color[2] = b;
}

void Node::increase_no_children() {
	children.push_back(NodePtr(new Node));
	children.back()->parent = this;
}

void Node::setup_frames(long frames) {
	//If the vectors are not empty then resize but fill with the value of the
	//last element
	if (scale.size() > 0) {
		scale.resize(frames, scale.back());
		froset->resize(frames, froset->back());
		freuler->resize(frames, freuler->back());
		quat_arr.resize(frames, quat_arr.back());
	} else {
		scale.resize(frames);
		froset->resize(frames);
		freuler->resize(frames);
		quat_arr.resize(frames);
	}
}

unsigned int Node::noofchildren() {
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

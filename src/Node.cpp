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
	color = osg::Vec4(0.5f, 0.5f, 0.5f, 1.0);
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
	color[0] = r;
	color[1] = g;
	color[2] = b;
}

void Node::increase_no_children() {
	children.push_back(NodePtr(new Node));
	children.back()->parent = this;
}

void Node::setup_frames(long frames) {
	scale.resize(frames);
	froset->resize(frames);
	freuler->resize(frames);
}

unsigned int Node::noofchildren() {
	return children.size();
}

Node* Node::get_last_child() {
	return children.back().get();
}

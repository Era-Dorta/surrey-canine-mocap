/*
 * Node.h
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */

#ifndef NODE_H_
#define NODE_H_

#include <osg/Array>
#include <osg/MatrixTransform>
#include <cstddef>
#include <vector>
#include <string>

typedef unsigned char BYTE;

class Node {
	public:
		Node();
		virtual ~Node();

		void increase_no_children();

		void setup_children(int new_no_children);

		void setup_offset(float x = 0.0f, float y = 0.0f, float z = 0.0f);

		void setup_euler(float r1 = 0.0f, float r2 = 0.0f, float r3 = 0.0f);

		void setup_color(float r = 0.5f, float g = 0.5f, float b = 0.5f);

		void setup_frames(long frames);

		std::string name;
		osg::Vec3f length;    // length of segment
		osg::Vec3f offset; // Transitional offset with respect to the end of the partent link
		osg::Vec3f euler;     // Rotation
		osg::Vec4 color;
		int noofchildren;
		std::vector<Node *> children;    // Array of pointers to child nodes
		Node *parent;       // Back pointer to parent node
		osg::ref_ptr<osg::Vec3Array> froset;  // Array of offsets for each frame
		osg::ref_ptr<osg::Vec3Array> freuler;  // Array of angles for each frame
		std::vector<float> scale;       // Array of scalefactors for each frame
		BYTE DOFs;          // Used to determine what DOFs the segment has
		int noofchannels;

		//TODO Better to have a list or some other kind of struture in controller
		//model should not know about the viewer
		osg::ref_ptr<osg::MatrixTransform> osg_node;
};

#endif /* NODE_H_ */

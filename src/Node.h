/*
 * Node.h
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */

#ifndef NODE_H_
#define NODE_H_

#include <osg/Array>
#include <cstddef>
#include <vector>
#include <string>
#include "DebugUtil.h"

typedef unsigned char BYTE;

class NODE {
	public:
		NODE();
		virtual ~NODE();
		std::string name;
		osg::Vec3f length;    // length of segment
		osg::Vec3f offset; // Transitional offset with respect to the end of the partent link
		osg::Vec3f euler;     // Rotation
		osg::Vec3f colour;
		int noofchildren;
		NODE **children;    // Array of pointers to child nodes
		NODE *parent;       // Back pointer to parent node
		osg::ref_ptr<osg::Vec3Array> froset;  // Array of offsets for each frame
		osg::ref_ptr<osg::Vec3Array> freuler;  // Array of angles for each frame
		std::vector<float> scale;       // Array of scalefactors for each frame
		BYTE DOFs;          // Used to determine what DOFs the segment has
		int noofchannels;
};

#endif /* NODE_H_ */

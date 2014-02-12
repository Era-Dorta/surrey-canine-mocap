/*
 * MocapHeader.h
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */

#ifndef MOCAPHEADER_H_
#define MOCAPHEADER_H_

#include <osg/Array>

class MocapHeader {
public:
	MocapHeader();
	virtual ~MocapHeader();

	int noofsegments; // Number of body segments
	long noofframes; // Number of frames
	int datarate; // Number of frames per second
	float frametime;
	osg::ref_ptr<osg::Vec3Array> euler; // Specifies how the euler angle is defined
	float callib; // Scale factor for converting current translational units into meters
	float inv_callib; // Inverse of Scale factor, useful when saving
	bool degrees; // Are the rotational measurements in degrees
	float scalefactor; // Global Scale factor
	long currentframe; // Stores the currentframe to render
};

#endif /* MOCAPHEADER_H_ */

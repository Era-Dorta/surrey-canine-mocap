/*
 * SkeletonState.h
 *
 *  Created on: 11 Dec 2013
 *      Author: m04701
 */

#ifndef SKELETONSTATE_H_
#define SKELETONSTATE_H_

#include "Skeleton.h"
#include "boost/shared_ptr.hpp"

class SkeletonState {
public:
	SkeletonState();
	virtual ~SkeletonState();
	void save_state(boost::shared_ptr<Skeleton> skeleton, int frame_num);
	void restore_state(boost::shared_ptr<Skeleton> skeleton, int frame_num);
private:
	void init(unsigned int size);

	std::vector<osg::Quat> rotations;
	osg::ref_ptr<osg::Vec3Array> offsets;
	osg::ref_ptr<osg::Vec3Array> lengths;
};

#endif /* SKELETONSTATE_H_ */

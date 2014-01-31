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

//TODO Update to save all the skeleton now that inverse kinematics move
//several bones at a time
class SkeletonState {
	public:
		SkeletonState();
		virtual ~SkeletonState();
		void save_state(boost::shared_ptr<Skeleton> skeleton, int frame_num,
				unsigned int node_index);
		void restore_state(boost::shared_ptr<Skeleton> skeleton, int frame_num,
				unsigned int node_index);
	private:
		osg::Quat node_rotation;
		osg::Vec3 node_offset;
		osg::Vec3 node_length;
		osg::Quat parent_rotation;
		osg::Vec3 parent_offset;
		osg::Vec3 parent_length;
		osg::ref_ptr<osg::Vec3Array> children_offset;
};

#endif /* SKELETONSTATE_H_ */

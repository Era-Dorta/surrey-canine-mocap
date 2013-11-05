/*
 * SkeletonFitting.h
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#ifndef SKELETONFITTING_H_
#define SKELETONFITTING_H_

#include "Skeleton.h"
#include <osg/Vec3>

class SkeletonFitting {
	public:
		SkeletonFitting();
		virtual ~SkeletonFitting();
		void fit_skeleton_into_cloud(Skeleton& skeleton, osg::ref_ptr<osg::Vec3Array> cloud);
		void fit_skeleton_with_prev_nex_frame(Skeleton& skeleton, int frame);
};

#endif /* SKELETONFITTING_H_ */

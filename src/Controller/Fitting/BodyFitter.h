/*
 * BodyFitter.h
 *
 *  Created on: 24 Feb 2014
 *      Author: m04701
 */

#ifndef BODYFITTER_H_
#define BODYFITTER_H_

#include "CommonFitter.h"

class BodyFitter: public CommonFitter {
public:
	BodyFitter(SkeletonPtr skeleton, Skeletonization3DPtr skeletonization3d,
			const camVecT& camera_arr);

	bool fit_root_position(const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels);

	int fit_head_and_back(const PointCloudPtr& cloud,
			const std::vector<Skeleton::Skel_Leg>& labels);
};

#endif /* BODYFITTER_H_ */

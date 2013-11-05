/*
 * SkeletonFitting.cpp
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"

SkeletonFitting::SkeletonFitting() :
			move_joint_max_dist(0) {
}

SkeletonFitting::~SkeletonFitting() {
}

void SkeletonFitting::fit_skeleton_into_cloud(Skeleton& skeleton,
		osg::ref_ptr<osg::Vec3Array> cloud) {

	//TODO
	//Select all points that are closer than a threshold to the line formed by
	//a pair of joints, care with points far away from the joints, case the line
	//cuts other points in the skeleton.

	//How to get the points???
	//Segment in boxes first???

	//Call fitLine with those points and the line formed by the joints
	//New joints shall be the closest points of the new line to the previous
	//joints

	//fitLine(InputArray points, OutputArray line, int distType, double param, double reps, double aeps)
	//points – Input vector of 2D or 3D points, stored in std::vector<> or Mat.
	//line – Output line parameters. In case of 2D fitting, it should be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
	//distType – Distance used by the M-estimator (see the discussion below).
	//param – Numerical parameter ( C ) for some types of distances. If it is 0, an optimal value is chosen.
	//reps – Sufficient accuracy for the radius (distance between the coordinate origin and the line).
	//aeps – Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.

}

void SkeletonFitting::fit_skeleton_with_prev_nex_frame(Skeleton& skeleton,
		int frame) {
	//TODO
	//Calculate distance to joint in next and previous frame
	//Calculate vectors of movement to positions in next and previous frame
	//Add them
	//If the are not 0 in any dimension, it means that if we move in that dimension
	//a certain amount, then distance to prev and next should be reduced.
	//Iterative??? How much to move???
}

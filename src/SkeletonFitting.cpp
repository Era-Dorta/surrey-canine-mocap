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

//void SkeletonFitting::fit_skeleton_into_cloud(Skeleton& skeleton,
//		osg::ref_ptr<osg::Vec3Array> cloud) {

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

//}

//void SkeletonFitting::fit_skeleton_with_prev_nex_frame(Skeleton& skeleton,
//	int frame) {
//TODO
//Calculate distance to joint in next and previous frame
//Calculate vectors of movement to positions in next and previous frame
//Add them
//If the are not 0 in any dimension, it means that if we move in that dimension
//a certain amount, then distance to prev and next should be reduced.
//Iterative??? How much to move???
//}

void SkeletonFitting::divide_four_sections(osg::ref_ptr<osg::Vec3Array> cloud,
		std::vector<Skel_Leg>& result) {
	result.clear();
	result.resize(cloud->size(), Front_Left);

	float mean_y = get_median(cloud, result, Front_Left, Y).y();

	//Divide in half vertically, discard all values above
	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (cloud->at(i).y() < mean_y) {
			result[i] = Not_Use;
		}
	}

	//Divide the remaining values in front/back part along x
	float mean_x = get_median(cloud, result, Front_Left, X).x();
	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (result[i] != Not_Use && cloud->at(i).x() <= mean_x) {
			result[i] = Back_Left;
		}
	}

	//Divide the two groups into left and right
	float mean_z_front = get_median(cloud, result, Front_Left, Z).z();
	float mean_z_back = get_median(cloud, result, Back_Left, Z).z();
	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (result[i] == Front_Left && cloud->at(i).z() >= mean_z_front) {
			result[i] = Front_Right;
		} else if (result[i] == Back_Left && cloud->at(i).z() >= mean_z_back) {
			result[i] = Back_Right;
		}
	}
}

bool comp_x(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.x() < j.x());
}

bool comp_y(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.y() < j.y());
}

bool comp_z(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.z() < j.z());
}

osg::Vec3 SkeletonFitting::get_median(osg::ref_ptr<osg::Vec3Array> points,
		std::vector<Skel_Leg>& result, Skel_Leg use_type, Axis axis) {
	osg::ref_ptr<osg::Vec3Array> aux_vec = new osg::Vec3Array();

	for (unsigned int i = 0; i < points->size(); i++) {
		if (result[i] == use_type) {
			aux_vec->push_back(points->at(i));
		}
	}

	osg::Vec3Array::iterator first = aux_vec->begin();
	osg::Vec3Array::iterator last = aux_vec->end();
	osg::Vec3Array::iterator middle = first + (last - first) / 2;

	switch (axis) {
	case X:
		std::nth_element(first, middle, last, comp_x);
		break;
	case Y:
		std::nth_element(first, middle, last, comp_y);
		break;
	case Z:
		std::nth_element(first, middle, last, comp_z);
		break;
	}

	osg::Vec3 res = *middle;
	return res;
}

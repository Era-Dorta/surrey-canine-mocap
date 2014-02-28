/*
 * DebugUtil.hpp
 *
 *  Created on: 12 Nov 2013
 *      Author: m04701
 */

#ifndef DEBUGUTIL_HPP_
#define DEBUGUTIL_HPP_

#include "Skeleton.h"
#include "CudaVec.h"
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Matrix>

#include <iostream>
using std::cout;
using std::endl;

inline std::ostream& operator<<(std::ostream &out, const osg::Vec3 &vector) {
	out << "[" << vector.x() << ", " << vector.y() << ", " << vector.z() << "]";
	return out;
}

inline std::ostream& operator<<(std::ostream &out, const osg::Vec4 &vector) {
	out << "[" << vector.x() << ", " << vector.y() << ", " << vector.z() << ", "
			<< vector.w() << "]";
	return out;
}

inline std::ostream& operator<<(std::ostream &out, const float3 &vector) {
	out << "[" << vector.x << ", " << vector.y << ", " << vector.z << "]";
	return out;
}

inline std::ostream& operator<<(std::ostream &out, const float4 &vector) {
	out << "[" << vector.x << ", " << vector.y << ", " << vector.z << ", "
			<< vector.w << "]";
	return out;
}

inline std::ostream& operator<<(std::ostream &out, const osg::Quat &quat) {
	out << "[" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", "
			<< quat.w() << "]";
	return out;
}

inline std::ostream& operator<<(std::ostream &out, const osg::Matrix &m) {
	out << endl;
	for (int i = 0; i < 4; i++) {
		out << "[";
		int j;
		for (j = 0; j < 3; j++) {
			out << m(i, j) << ", ";
		}
		out << m(i, j) << "]" << endl;
	}
	return out;
}

inline std::ostream& operator<<(std::ostream &out,
		const Skeleton::Skel_Leg &leg) {
	switch (leg) {
	case Skeleton::Front_Left:
		out << "Front_Left";
		break;
	case Skeleton::Front_Right:
		out << "Front_Right";
		break;
	case Skeleton::Back_Left:
		out << "Back_Left";
		break;
	case Skeleton::Back_Right:
		out << "Back_Right";
		break;
	case Skeleton::Not_Limbs:
		out << "Not_Limbs";
		break;
	}
	return out;
}

#endif /* DEBUGUTIL_HPP_ */

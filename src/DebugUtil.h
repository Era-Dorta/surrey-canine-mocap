/*
 * DebugUtil.hpp
 *
 *  Created on: 12 Nov 2013
 *      Author: m04701
 */

#ifndef DEBUGUTIL_HPP_
#define DEBUGUTIL_HPP_

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

#endif /* DEBUGUTIL_HPP_ */

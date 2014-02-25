/*
 * MiscUtils.h
 *
 *  Created on: 25 Feb 2014
 *      Author: m04701
 */

#ifndef COMPMETHODS_H_
#define COMPMETHODS_H_

namespace CompMethods {

inline bool comp_x(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.x() < j.x());
}

inline bool comp_y(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.y() < j.y());
}

inline bool comp_z(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.z() < j.z());
}
}

#endif /* COMPMETHODS_H_ */

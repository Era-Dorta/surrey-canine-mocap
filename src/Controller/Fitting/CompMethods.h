/*
 * MiscUtils.h
 *
 *  Created on: 25 Feb 2014
 *      Author: m04701
 */

#ifndef COMPMETHODS_H_
#define COMPMETHODS_H_

#include <pcl/point_types.h>

namespace CompMethods {

inline bool comp_x(const pcl::PointXYZ& i, const pcl::PointXYZ& j) {
	return (i.x < j.x);
}

inline bool comp_y(const pcl::PointXYZ& i, const pcl::PointXYZ& j) {
	return (i.y < j.y);
}

inline bool comp_z(const pcl::PointXYZ& i, const pcl::PointXYZ& j) {
	return (i.z < j.z);
}
}

#endif /* COMPMETHODS_H_ */

/*
 * SkeletonFitting.h
 *
 *  Created on: 1 Nov 2013
 *      Author: m04701
 */

#ifndef SKELETONFITTING_H_
#define SKELETONFITTING_H_

#include <osg/Vec3>
#include <osg/Group>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <fstream>
#include <iostream>
#include <string>
using std::cout;
using std::endl;


class SkeletonFitting {
	public:
		SkeletonFitting();
		virtual ~SkeletonFitting();
		void add_joint(osg::Vec3& joint);
		void move_joint(unsigned int index, osg::Vec3& new_pos);
		void delete_joint(unsigned int index);
		osg::Vec3 get_joint(unsigned int index);
		unsigned int get_max_joints();
		bool skeleton_full();
	private:
		osg::ref_ptr<osg::Vec3Array> joint_array;
		unsigned int max_joints;
		void save_to_file();
		void read_from_file();
};

#endif /* SKELETONFITTING_H_ */

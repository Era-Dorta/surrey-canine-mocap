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
#include "MiscUtils.h"
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
		unsigned int get_num_joints();
		unsigned int get_max_joints();
		bool skeleton_full();
		void save_to_file(std::string file_name);
		void load_from_file(std::string file_name);
	private:
		osg::ref_ptr<osg::Vec3Array> joint_array;
		unsigned int max_joints;
};

#endif /* SKELETONFITTING_H_ */

/*
 * SkeletonFitting.h
 *
 *  Created on: 1 Nov 2013
 *      Author: m04701
 */

#ifndef SKELETON_H_
#define SKELETON_H_

#include <osg/Vec3>
#include <osg/Group>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <fstream>
#include <iostream>
#include <string>
#include "BVHFormat.h"

using std::cout;
using std::endl;

class Skeleton {
	public:
		Skeleton();
		virtual ~Skeleton();
		void add_joint(osg::Vec3& joint);
		void move_joint(unsigned int index, osg::Vec3& new_pos);
		void delete_joint(unsigned int index);
		const osg::Vec3& get_joint(unsigned int index);
		unsigned int get_num_joints();
		unsigned int get_max_joints();
		unsigned int get_num_bones();
		void get_bone(unsigned int index, osg::Vec3& i_pos, osg::Vec3& e_pos);
		bool skeleton_full();
		void save_to_file(std::string file_name);
		void load_from_file(std::string file_name);
		void set_current_frame(int frame_no);
		const osg::ref_ptr<osg::Vec3Array> getJointArray() const;
		Node* get_root();
		MOCAPHEADER& get_header();

	private:
		void reset_state();

		osg::ref_ptr<osg::Vec3Array> joint_array;
		std::vector<osg::ref_ptr<osg::Vec3Array> > joint_frame_array;
		std::vector<std::pair<int, int> > bone_array;
		unsigned int max_joints;
		int current_frame;
		BVHFormat bvhf;
		MOCAPHEADER mocap_header;
};

#endif /* SKELETON_H_ */

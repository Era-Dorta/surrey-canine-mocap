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

class Skeleton: public BVHFormat {
	public:
		Skeleton();
		virtual ~Skeleton();
		void rotate_joint(unsigned int index, osg::Vec3& angle);
		void rotate_every_frame(osg::Vec3& angle);
		void translate_root(osg::Vec3& translation);
		void translate_every_frame(osg::Vec3& translation);
		void change_color(int index, osg::Vec4 color);

		unsigned int get_num_bones();
		void save_to_file(std::string file_name);
		void load_from_file(std::string file_name);
		void set_current_frame(int frame_no);
		Node* get_root();
		MocapHeader& get_header();
		int get_node(osg::ref_ptr<osg::MatrixTransform> node_transform);
		bool isSkelLoaded() const;

	private:
		void reset_state();
		MocapHeader mocap_header;
		bool skel_loaded;
		float rotate_scale_factor;
		float translate_scale_factor;
};

#endif /* SKELETON_H_ */

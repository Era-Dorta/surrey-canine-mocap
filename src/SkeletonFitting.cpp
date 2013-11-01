/*
 * SkeletonFitting.cpp
 *
 *  Created on: 1 Nov 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"

SkeletonFitting::SkeletonFitting():
	max_joints(3) {
	joint_array = new osg::Vec3Array();
}

SkeletonFitting::~SkeletonFitting() {
	// TODO Auto-generated destructor stub
}

void SkeletonFitting::add_joint(osg::Vec3& joint) {
	if(joint_array->size() < max_joints){
		joint_array->push_back(joint);
		save_to_file();
	}else{
		save_to_file();
	}
}

void SkeletonFitting::move_joint(unsigned int index, osg::Vec3& new_pos) {
	(*joint_array)[index] = new_pos;
}

void SkeletonFitting::delete_joint(unsigned int index) {
	joint_array->erase(joint_array->begin() + index );
}

osg::Vec3 SkeletonFitting::get_joint(unsigned int index) {
	return (*joint_array)[index];
}

unsigned int SkeletonFitting::get_max_joints() {
	return max_joints;
}

bool SkeletonFitting::skeleton_full() {
	return max_joints == joint_array->size();
}

void SkeletonFitting::save_to_file() {
	std::ofstream out_file;
	out_file.open("joint_points.txt");

	out_file << joint_array->size() << endl;
	for(unsigned int i = 0; i < joint_array->size(); i++){
		out_file << (*joint_array)[i][0] << endl;
		out_file << (*joint_array)[i][1] << endl;
		out_file << (*joint_array)[i][2] << endl;
	}
	out_file.close();
}

void SkeletonFitting::read_from_file() {
	std::ifstream in_file;
	in_file.open("joint_points.txt");
	if (in_file.is_open())
	{
		joint_array->clear();
		std::string line;
		unsigned int max_lines = atoi( line.c_str() );
		float x,y,z;
		for(unsigned int i = 0; i < max_lines; i++){

			std::getline (in_file,line);
			x = atof( line.c_str() );
			std::getline (in_file,line);
			y = atof( line.c_str() );
			std::getline (in_file,line);
			z = atof( line.c_str() );

			osg::Vec3 point(x,y,z);
			joint_array->push_back(point);
		}
	}
	in_file.close();
}

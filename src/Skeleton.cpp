/*
 * SkeletonFitting.cpp
 *
 *  Created on: 1 Nov 2013
 *      Author: m04701
 */

#include "Skeleton.h"

Skeleton::Skeleton() :
			current_frame(0) {

	SetHeader(&mocap_header);
	ImportData("/home/cvssp/misc/m04701/workspace/data/bvh/Dog_modelling.bvh");
	//"/home/cvssp/misc/m04701/workspace/data/bvh/example.bvh");
	//bvhf.ExportData("/home/cvssp/misc/m04701/workspace/data/bvh/out.bvh");

	max_joints = mocap_header.noofsegments;

	bone_array.reserve(18);

	joint_array = new osg::Vec3Array;
	//Right front Leg
	bone_array.push_back(std::make_pair(0, 1));
	bone_array.push_back(std::make_pair(1, 2));
	bone_array.push_back(std::make_pair(2, 3));
	bone_array.push_back(std::make_pair(3, 4));

	//Left front Leg
	bone_array.push_back(std::make_pair(4, 5));
	bone_array.push_back(std::make_pair(5, 6));
	bone_array.push_back(std::make_pair(6, 7));
	bone_array.push_back(std::make_pair(7, 8));

	//Right back Leg
	bone_array.push_back(std::make_pair(9, 10));
	bone_array.push_back(std::make_pair(10, 11));
	bone_array.push_back(std::make_pair(11, 12));
	bone_array.push_back(std::make_pair(12, 13));

	//Left back Leg
	bone_array.push_back(std::make_pair(13, 14));
	bone_array.push_back(std::make_pair(14, 15));
	bone_array.push_back(std::make_pair(15, 16));
	bone_array.push_back(std::make_pair(16, 17));

	//Head
	bone_array.push_back(std::make_pair(4, 19));
	bone_array.push_back(std::make_pair(18, 19));

	//Back
	bone_array.push_back(std::make_pair(4, 13));

	//Tail
	bone_array.push_back(std::make_pair(13, 20));
}

Skeleton::~Skeleton() {
	// TODO Auto-generated destructor stub
}

void Skeleton::add_joint(osg::Vec3& joint) {
	if (joint_array->size() < max_joints) {
		joint_array->push_back(joint);
	}
}

void Skeleton::rotate_joint(unsigned int index, osg::Vec3& angle) {
	angle *= 0.1;
	nodelist[index]->freuler->at(current_frame) += angle;
}

void Skeleton::translate_root(osg::Vec3& translation) {
	translation *= 0.005;
	root->offset += translation;
}

void Skeleton::delete_joint(unsigned int index) {
	joint_array->erase(joint_array->begin() + index);
}

const osg::Vec3& Skeleton::get_joint(unsigned int index) {
	return (*joint_array)[index];
}

unsigned int Skeleton::get_max_joints() {
	return max_joints;
}

bool Skeleton::skeleton_full() {
	return max_joints == joint_array->size();
}

void Skeleton::save_to_file(std::string file_name) {
	std::ofstream out_file;
	out_file.open(file_name.c_str());

	out_file << joint_frame_array.size() << endl;
	for (unsigned int i = 0; i < joint_frame_array.size(); i++) {
		joint_array = joint_frame_array[i];
		if (joint_array.valid()) {
			out_file << joint_array->size() << endl;
			for (unsigned int j = 0; j < joint_array->size(); j++) {
				out_file << (*joint_array)[j][0] << endl;
				out_file << (*joint_array)[j][1] << endl;
				out_file << (*joint_array)[j][2] << endl;
			}
		} else {
			out_file << "0" << endl;
		}
	}
	out_file.close();
}

unsigned int Skeleton::get_num_joints() {
	return joint_array->size();
}

void Skeleton::load_from_file(std::string file_name) {
	std::ifstream in_file;
	in_file.open(file_name.c_str());
	if (in_file.is_open()) {
		if (joint_array.valid()) {
			joint_array->clear();
		}
		joint_frame_array.clear();
		std::string line;
		std::getline(in_file, line);
		unsigned int num_frames = atoi(line.c_str());
		joint_frame_array.resize(num_frames);
		float x, y, z;

		for (unsigned int i = 0; i < num_frames; i++) {
			std::getline(in_file, line);
			unsigned int num_vectors = atoi(line.c_str());

			joint_array = new osg::Vec3Array;

			for (unsigned int j = 0; j < num_vectors; j++) {

				std::getline(in_file, line);
				x = atof(line.c_str());
				std::getline(in_file, line);
				y = atof(line.c_str());
				std::getline(in_file, line);
				z = atof(line.c_str());

				osg::Vec3 point(x, y, z);
				joint_array->push_back(point);
			}
			joint_frame_array[i] = joint_array;
		}
	}
	in_file.close();

	reset_state();
}

unsigned int Skeleton::get_num_bones() {
	return bone_array.size();
}

void Skeleton::get_bone(unsigned int index, osg::Vec3& i_pos,
		osg::Vec3& e_pos) {
	i_pos = (*joint_array)[bone_array[index].first];
	e_pos = (*joint_array)[bone_array[index].second];
}

void Skeleton::set_current_frame(int frame_no) {
	current_frame = frame_no;

	//Frame beyond vector
	if (current_frame >= joint_frame_array.size()) {
		joint_frame_array.resize(current_frame + 1);
	}

	//Joint no still initialised
	if (!joint_frame_array[current_frame].valid()) {
		joint_frame_array[current_frame] = new osg::Vec3Array();
	}

	joint_array = joint_frame_array[current_frame];
}

Node* Skeleton::get_root() {
	return GetRootNode();
}

MocapHeader& Skeleton::get_header() {
	return mocap_header;
}

void Skeleton::reset_state() {
	set_current_frame(current_frame);
}

const osg::ref_ptr<osg::Vec3Array> Skeleton::getJointArray() const {
	return joint_array;
}

int Skeleton::get_node(osg::ref_ptr<osg::MatrixTransform> node_transform) {
	for (unsigned int i = 0; i < nodelist.size(); i++) {
		if (nodelist[i]->osg_node.get() == node_transform.get()) {
			return i;
		}
	}
	return -1;
}

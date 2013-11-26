/*
 * SkeletonMixer.cpp
 *
 *  Created on: 25 Nov 2013
 *      Author: m04701
 */

#include "SkeletonMixer.h"
#include "DebugUtil.h"

SkeletonMixer::SkeletonMixer() {

}

SkeletonMixer::SkeletonMixer(std::vector<std::string>& file_names,
		int start_frame) {
	set_data(file_names, start_frame);
}

SkeletonMixer::~SkeletonMixer() {

}

void SkeletonMixer::set_data(std::vector<std::string>& file_names,
		int start_frame) {
	skel_arr.clear();
	skel_arr.resize(file_names.size());
	for (unsigned int i = 0; i < file_names.size(); i++) {
		skel_arr.at(i).load_from_file(file_names.at(i));
	}
	skel_result.load_from_file(file_names.front());
}

void SkeletonMixer::mix() {

	float inv_size = 1.0 / skel_arr.size();
	for (unsigned int i = 0; i < skel_result.get_num_bones(); i++) {
		float dist2 = 0.0;

		//Accumulate the square distance of all the other bone samples
		std::vector<Skeleton>::iterator skeleton = skel_arr.begin();
		for (; skeleton != skel_arr.end(); ++skeleton) {
			Node* other_bone = skeleton->get_node(i);
			dist2 += other_bone->length.length2();
		}

		//Divide by number of samples
		dist2 *= inv_size;
		Node* bone = skel_result.get_node(i);

		//New length is calculated by solving euclidean distance for x, y, z
		//We left x and z with their previous values and calculate a new value
		//for y that it is a satisfies the mean distance
		float y = std::sqrt(
				dist2 - bone->length.x() * bone->length.x()
						- bone->length.z() * bone->length.z());

		//If the node is not a leaf the update all its children positions
		for (unsigned int j = 0; j < bone->get_num_children(); j++) {
			bone->children[j]->offset.y() = y;
		}
		bone->length.y() = y;
	}
}

void SkeletonMixer::save_file(std::string& file_name) {
	skel_result.save_to_file(file_name);
}

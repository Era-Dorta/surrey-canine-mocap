/*
 * SkeletonMixer.cpp
 *
 *  Created on: 25 Nov 2013
 *      Author: m04701
 */

#include "SkeletonMixer.h"

SkeletonMixer::SkeletonMixer() {

}

SkeletonMixer::SkeletonMixer(std::string& model,
		std::vector<std::string>& to_mix) {
	set_data(model, to_mix);
}

SkeletonMixer::~SkeletonMixer() {

}

void SkeletonMixer::set_data(std::string& model,
		std::vector<std::string>& to_mix) {
	skel_arr.clear();
	skel_arr.resize(to_mix.size());
	for (unsigned int i = 0; i < to_mix.size(); i++) {
		skel_arr.at(i).load_from_file(to_mix.at(i));
	}
	skel_result.load_from_file(model);
}

void SkeletonMixer::mix() {

	float inv_size = 1.0 / skel_arr.size();
	for (unsigned int i = 0; i < skel_result.get_num_bones(); i++) {
		float new_dist = 0.0;

		//Accumulate the square distance of all the other bone samples
		std::vector<Skeleton>::iterator skeleton = skel_arr.begin();
		for (; skeleton != skel_arr.end(); ++skeleton) {
			Node* other_bone = skeleton->get_node(i);
			new_dist += other_bone->length.length2();
		}

		//Better to calculate the square root only once in the end that for
		//every other bone sample
		new_dist = std::sqrt(new_dist);

		//Divide by number of samples
		new_dist *= inv_size;

		Node* bone = skel_result.get_node(i);

		//New position is solving euclidean distance, but we want a factor
		//that multiplies all the values to grow/decrease along the bone direction
		// sqrt( (x*new_prop)^2 + (x*new_prop)^2 + (x*new_prop)^2 ) = new_dist
		double new_prop = new_dist / (double) bone->length.length();

		//If the node is not a leaf the update all its children positions
		for (unsigned int j = 0; j < bone->get_num_children(); j++) {
			bone->children[j]->offset *= new_prop;
		}
		bone->length *= new_prop;
	}
}

void SkeletonMixer::save_file(std::string& file_name) {
	skel_result.save_to_file(file_name);
}

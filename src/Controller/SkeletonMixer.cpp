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
	init(model, to_mix);
}

SkeletonMixer::~SkeletonMixer() {

}

void SkeletonMixer::init(std::string& model, std::vector<std::string>& to_mix) {
	skel_arr.clear();
	skel_arr.resize(to_mix.size());
	for (unsigned int i = 0; i < to_mix.size(); i++) {
		skel_arr.at(i).load_from_file(to_mix.at(i));
	}
	skel_result.load_from_file(model);

	//Make sure all rotations are set to 0, we are going to give out a
	//clean 0 rotations file
	for (unsigned int i = 0; i < skel_result.get_num_bones(); i++) {
		Node* node = skel_result.get_node(i);
		for (unsigned int j = 0; j < node->quat_arr.size(); j++) {
			node->quat_arr.at(j).set(0, 0, 0, 1);
		}
	}

	//Also set all per frame translations to 0
	Node* root = skel_result.get_root();
	for (unsigned int i = 0; i < root->froset->size(); i++) {
		root->froset->at(i).set(0, 0, 0);
	}
}

void SkeletonMixer::mix() {

	float inv_size = 1.0 / skel_arr.size();
	for (unsigned int i = 0; i < skel_result.get_num_bones(); i++) {
		float new_dist = 0.0;

		//Accumulate the distance of all the other bone samples
		std::vector<Skeleton>::iterator skeleton = skel_arr.begin();
		for (; skeleton != skel_arr.end(); ++skeleton) {
			Node* other_bone = skeleton->get_node(i);
			new_dist += other_bone->get_length();
		}

		//Divide by number of samples
		new_dist *= inv_size;

		update_bone_length(i, new_dist);
	}

	//Make right and left legs to have the same bone size
	int br, er, bl, el;
	er = Skeleton::Front_Right;
	br = er - 3;
	el = Skeleton::Front_Left;
	bl = el - 3;
	mix_right_left_leg(br, er, bl, el);

	er = Skeleton::Back_Right;
	br = er - 3;
	el = Skeleton::Back_Left;
	bl = el - 3;
	mix_right_left_leg(br, er, bl, el);
}

void SkeletonMixer::save_file(std::string& file_name) {
	skel_result.save_to_file(file_name);
}

void SkeletonMixer::mix_right_left_leg(unsigned int start_right,
		unsigned int end_right, unsigned int start_left,
		unsigned int end_left) {

	unsigned int right_index = start_right;
	for (unsigned int left_index = start_left; left_index <= end_left;
			left_index++) {
		float new_dist = 0.0;
		//Accumulate the distance of right and left leg current bone
		Node* other_bone = skel_result.get_node(left_index);
		new_dist += other_bone->get_length();
		other_bone = skel_result.get_node(right_index);
		new_dist += other_bone->get_length();

		//Get the mean
		new_dist *= 0.5;

		//Update the to bones lengths
		update_bone_length(right_index, new_dist);
		update_bone_length(left_index, new_dist);

		right_index++;
	}
}

void SkeletonMixer::update_bone_length(unsigned int index, double distance) {
	Node* bone = skel_result.get_node(index);

	//New position is solving euclidean distance, but we want a factor
	//that multiplies all the values to grow/decrease along the bone direction
	// sqrt( (x*new_prop)^2 + (x*new_prop)^2 + (x*new_prop)^2 ) = new_dist
	double new_prop = distance / (double) bone->get_length();

	//If the node is not a leaf the update all its children positions
	for (unsigned int j = 0; j < bone->get_num_children(); j++) {
		bone->children[j]->set_offset(
				bone->children[j]->get_offset() * new_prop);
	}
	bone->set_local_end(bone->get_local_end() * new_prop);
}

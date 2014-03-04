/*
 * SkeletonMixer.h
 *
 *  Created on: 25 Nov 2013
 *      Author: m04701
 */

#ifndef SKELETONMIXER_H_
#define SKELETONMIXER_H_

#include "../Model/Skeleton.h"
#include "vector"

class SkeletonMixer {
public:
	SkeletonMixer();

	//model: file containing a skeleton in a neutral position
	//to_mix: vector of skeleton files that are going to be mixed
	SkeletonMixer(std::string& model, std::vector<std::string>& to_mix);

	virtual ~SkeletonMixer();

	void set_data(std::string& model, std::vector<std::string>& to_mix);

	//Mix all the data. Bones sizes of model are the mean size of to_mix
	//skeletons. If empty constructor was called and set data
	//was not called, then is does nothing
	void mix();

	//Save result in file_name
	void save_file(std::string& file_name);

private:
	std::vector<Skeleton> skel_arr;
	Skeleton skel_result;
};

#endif /* SKELETONMIXER_H_ */

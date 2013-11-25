/*
 * SkeletonMixer.h
 *
 *  Created on: 25 Nov 2013
 *      Author: m04701
 */

#ifndef SKELETONMIXER_H_
#define SKELETONMIXER_H_

#include "Skeleton.h"
#include "vector"

class SkeletonMixer {
	public:
		SkeletonMixer();
		virtual ~SkeletonMixer();

		void set_data(std::vector<std::string>& file_names, int start_frame);
		void mix();
		void save_file();

	private:
		std::vector<Skeleton> skel_arr;
		Skeleton skel_result;
};

#endif /* SKELETONMIXER_H_ */

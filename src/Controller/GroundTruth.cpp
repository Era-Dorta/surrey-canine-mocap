/*
 * GroundTruth.cpp
 *
 *  Created on: 7 Mar 2014
 *      Author: m04701
 */

#include "GroundTruth.h"

GroundTruth::GroundTruth(unsigned int max_user_points) {
	data_loaded = false;
	this->max_user_points = max_user_points;
}

void GroundTruth::set_ground_truth_frame(const std::vector<osg::Vec3>& points,
		unsigned int frame_num) {

	if (frame_num >= points_vec.size()
			|| points_vec[frame_num].size() < points.size()) {
		unsigned int prev_size = points_vec.size();
		points_vec.resize(frame_num + 1);

		//If there is a gap between this and previous frame at least
		//initialise previous frames to 0
		for (unsigned int i = prev_size; i <= frame_num; i++) {
			points_vec[i].resize(max_user_points);
		}
	}

	std::vector<osg::Vec3>& points_frame = points_vec[frame_num];
	int index_offset = 0;
	for (unsigned int i = 0; i < points.size(); i++) {
		points_frame[i + index_offset] = points[i];

		//We don't have information on bones 2, 6, 11 and 15 so
		//leave them at (0, 0, 0)
		if (i == 1 || i == 4 || i == 8 || i == 11) {
			index_offset++;
		}
	}
}

bool GroundTruth::save_data(const std::string& path) {
	std::ofstream out_file;
	out_file.open(path.c_str());

	if (out_file.is_open()) {
		try {
			//Set so that all float numbers are written with 6 decimals
			out_file.precision(6);
			out_file.setf(std::ios::fixed, std::ios::floatfield);
			out_file << points_vec.size() << endl;
			if (points_vec.size() > 0) {
				out_file << points_vec.front().size() << endl;
				save_points(out_file);
			}
		} catch (...) {
			cout << "Error when saving ground truth" << endl;
			out_file.close();
			throw;
		}
		out_file.close();
		return true;
	} else {
		cout << "Could not open file to save ground truth";
		return false;
	}
}

bool GroundTruth::load_data(const std::string& path) {
	data_loaded = false;
	std::ifstream in_file;
	in_file.open(path.c_str());

	if (!in_file.is_open()) {
		cout << "Could not open file to load ground truth";
		return false;
	}

	try {
		points_vec.clear();
		std::string line;
		unsigned int num_frames = 0, num_points_frame = 0;

		if (std::getline(in_file, line)) {
			std::stringstream ss(line);
			ss >> num_frames;
		}

		if (std::getline(in_file, line)) {
			std::stringstream ss(line);
			ss >> num_points_frame;
			if (num_points_frame != max_user_points) {
				cout << "Ground Truth file must have 20 points "
						<< num_points_frame << endl;
				return false;
			}
		}
		load_points(in_file, num_frames);
	} catch (...) {
		cout << "Error when loading ground truth" << endl;
		in_file.close();
		return false;
	}
	in_file.close();
	data_loaded = true;
	return true;
}

float GroundTruth::calculate_total_error_skeleton(const SkeletonPtr skeleton) {
	if (points_vec.size() == 0) {
		return 0;
	}
	float skeleton_error = 0;
	for (unsigned int i = 0; i < points_vec.size(); i++) {
		skeleton_error = calculate_frame_error_skeleton(skeleton, i);
	}

	return skeleton_error / points_vec.size();
}

float GroundTruth::calculate_frame_error_skeleton(const SkeletonPtr skeleton,
		int frame_num) {
	float skeleton_error = 0;
	skeleton_error = calculate_root_start_frame_bone_error(skeleton, frame_num);
	for (unsigned int i = 0; i < skeleton->get_num_bones(); i++) {
		skeleton_error = calculate_frame_bone_error(skeleton, i, frame_num);
	}

	return skeleton_error / (skeleton->get_num_bones() + 1);
}

float GroundTruth::calculate_frame_bone_error(const SkeletonPtr skeleton,
		unsigned int bone_index, int frame) {

	float bone_error = 0;

	//We do not have information on this bones so error is 0
	if (bone_index == 2 || bone_index == 6 || bone_index == 11
			|| bone_index == 15) {
		return 0;
	}

	osg::Vec3 aux = skeleton->get_node(bone_index)->get_end_bone_global_pos(
			frame);

	aux = aux - points_vec[frame][bone_index + 1];

	bone_error = aux.length();

	return bone_error;
}

float GroundTruth::calculate_root_start_frame_bone_error(
		const SkeletonPtr skeleton, int frame) {

	osg::Vec3 bone_current_pos = skeleton->get_node(0)->get_global_pos(frame);

	float bone_error = (bone_current_pos - points_vec[frame][0]).length();

	return bone_error;
}

bool GroundTruth::is_data_loaded() const {
	return data_loaded;
}

void GroundTruth::save_points(std::ofstream& out_file) {
	Vec3DVec::const_iterator i = points_vec.begin();
	for (; i != points_vec.end(); ++i) {
		std::vector<osg::Vec3>::const_iterator j = i->begin();
		for (; j != i->end() - 1; ++j) {
			out_file << j->x() << " " << j->y() << " " << j->z() << " ";
		}
		out_file << j->x() << " " << j->y() << " " << j->z() << std::endl;
	}
}

const osg::Vec3& GroundTruth::get_point(unsigned int index,
		unsigned int frame_num) {
	if (frame_num < points_vec.size() && index < points_vec[frame_num].size()) {
		return points_vec[frame_num][index];
	}
	return empty;
}

void GroundTruth::load_points(std::ifstream& in_file, int num_frames) {
	points_vec.resize(num_frames);
	std::string line;

	for (int i = 0; i < num_frames; i++) {
		std::vector<osg::Vec3>& points = points_vec[i];
		points.resize(max_user_points);
		if (std::getline(in_file, line)) {
			std::stringstream ss(line);
			for (unsigned int j = 0; j < max_user_points; j++) {
				float x, y, z;
				x = read_float(ss);
				y = read_float(ss);
				z = read_float(ss);
				points[j].set(x, y, z);
			}
		}
	}
}

float GroundTruth::read_float(std::stringstream& ss) {
	float aux;
	if (ss >> aux) {
		return aux;
	}
	throw 20;
}

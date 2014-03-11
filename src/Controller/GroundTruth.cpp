/*
 * GroundTruth.cpp
 *
 *  Created on: 7 Mar 2014
 *      Author: m04701
 */

#include "GroundTruth.h"

GroundTruth::GroundTruth() {
	data_loaded = false;
}

void GroundTruth::set_ground_truth_frame(const std::vector<osg::Vec3>& points,
		unsigned int frame_num) {
	if (points_vec.size() <= frame_num) {
		unsigned int prev_size = points_vec.size();
		points_vec.resize(frame_num + 1);

		//If there is a gap between this and previous frame at least
		//initialise previous frames to 0
		for (unsigned int i = prev_size; i <= frame_num; i++) {
			points_vec[i].resize(points.size());
		}
	}

	std::vector<osg::Vec3>& points_frame = points_vec[frame_num];
	for (unsigned int i = 0; i < points.size(); i++) {
		points_frame[i] = points[i];
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
	std::ifstream in_file;
	in_file.open(path.c_str());

	if (in_file.is_open()) {
		try {
			points_vec.clear();
			std::string line;
			int num_frames = 0, num_points_frame = 0;

			if (std::getline(in_file, line)) {
				std::stringstream ss(line);
				ss >> num_frames;
			}

			if (std::getline(in_file, line)) {
				std::stringstream ss(line);
				ss >> num_points_frame;
			}
			load_points(in_file, num_frames, num_points_frame);
			data_loaded = true;
		} catch (...) {
			cout << "Error when loading ground truth" << endl;
			in_file.close();
			data_loaded = false;
			return false;
		}
		in_file.close();
		data_loaded = true;
		return true;
	} else {
		cout << "Could not open file to load ground truth";
		data_loaded = false;
		return false;
	}
}

float GroundTruth::calculate_total_error_skeleton(const SkeletonPtr skeleton) {
}

float GroundTruth::calculate_frame_error_skeleton(const SkeletonPtr skeleton,
		int frame_num) {
}

float GroundTruth::calculate_frame_bone_error(const SkeletonPtr skeleton,
		unsigned int bone_index, int frame) {
}

bool GroundTruth::is_data_loaded() const {
	return data_loaded;
}

void GroundTruth::save_points(std::ofstream& out_file) {
	std::vector<Vec3Vec>::const_iterator i = points_vec.begin();
	for (; i != points_vec.end(); ++i) {
		Vec3Vec::const_iterator j = i->begin();
		for (; j != i->end() - 1; ++j) {
			out_file << j->x() << " " << j->y() << " " << j->z() << " ";
		}
		out_file << j->x() << " " << j->y() << " " << j->z() << std::endl;
	}
}

void GroundTruth::load_points(std::ifstream& in_file, int num_frames,
		int num_points_frame) {
	points_vec.resize(num_frames);
	std::string line;

	for (int i = 0; i < num_frames; i++) {
		std::vector<osg::Vec3>& points = points_vec[i];
		points.resize(num_points_frame);
		if (std::getline(in_file, line)) {
			std::stringstream ss(line);
			for (int j = 0; j < num_points_frame; j++) {
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

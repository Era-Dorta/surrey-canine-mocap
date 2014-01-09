/*
 * CameraCalibrator.cpp
 *
 *  Created on: 8 Jan 2014
 *      Author: m04701
 */

#include "CameraCalibrator.h"

using std::cout;
using std::endl;

CameraCalibrator::CameraCalibrator() {

}

CameraCalibrator::CameraCalibrator(camVecT camera_arr_) {
	init(camera_arr_);
}

CameraCalibrator::~CameraCalibrator() {

}

void CameraCalibrator::init(camVecT camera_arr_) {
	camera_arr = camera_arr_;
}

void CameraCalibrator::set_plate_points(const osg::Vec3& p0,
		const osg::Vec3& p1, const osg::Vec3& p2, const osg::Vec3& p3) {

	osg::Vec3 center;
	center.x() = (p0.x() + p1.x() + p2.x() + p3.x()) / 4.0;
	center.y() = (p0.y() + p1.y() + p2.y() + p3.y()) / 4.0;
	center.z() = (p0.z() + p1.z() + p2.z() + p3.z()) / 4.0;

	calib_matrix = osg::Matrix::translate(-center);
}

void CameraCalibrator::save_camera_calibration(std::string path) {
	std::string T_fn(
			path + "/" + camera_arr.at(1)->get_cam_name()
					+ "/Calibration/T_rgb.txt");

	std::ofstream T_file;
	T_file.open(T_fn.c_str());

	//The matrix is to be saved transposed because when it is read
	//it will be transposed again
	if (T_file.is_open()) {
		for (int i = 0; i < 4; i++) {
			T_file << calib_matrix(0, i) << "\t";
			T_file << calib_matrix(1, i) << "\t";
			T_file << calib_matrix(2, i) << "\t";
			T_file << calib_matrix(3, i) << endl;
		}
		T_file.close();
	} else {
		cout << "Error: Failed to save extrinsic calibration file" << endl;
	}
}

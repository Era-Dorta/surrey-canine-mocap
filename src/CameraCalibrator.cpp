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

	for (camVecIte cam = camera_arr.begin(); cam != camera_arr.end(); ++cam) {
		std::string T_fn(
				path + "/" + (*cam)->get_cam_name()
						+ "/Calibration/T_rgb.txt");

		std::ofstream T_file;
		T_file.precision(20);
		T_file.open(T_fn.c_str());

		//The matrix is to be saved transposed because when it is read
		//it will be transposed again
		osg::Matrix aux_matrix = calib_matrix * (*cam)->get_T_osg();

		if (T_file.is_open()) {
			for (int i = 0; i < 4; i++) {
				T_file << aux_matrix(0, i) << "\t";
				T_file << aux_matrix(1, i) << "\t";
				T_file << aux_matrix(2, i) << "\t";
				T_file << aux_matrix(3, i) << endl;
			}
			T_file.close();
		} else {
			cout << "Error: Failed to save extrinsic calibration file" << endl;
		}
	}
}

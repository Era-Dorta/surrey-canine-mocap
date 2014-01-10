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

//Point order should be
//   p1____ p2
//     |   |
//     |   |
//	 p0|___|p3
void CameraCalibrator::set_plate_points(const osg::Vec3& p0,
		const osg::Vec3& p1, const osg::Vec3& p2, const osg::Vec3& p3) {

	//Use the user points to calculate new origin coordinates

	//First calculate translation, for that calculate the centre of the
	//points
	osg::Vec3 center;
	center.x() = (p0.x() + p1.x() + p2.x() + p3.x()) / 4.0;
	center.y() = (p0.y() + p1.y() + p2.y() + p3.y()) / 4.0;
	center.z() = (p0.z() + p1.z() + p2.z() + p3.z()) / 4.0;

	osg::Vec3 x_axis, y_axis, z_axis;

	//Then calculate rotations, use two vector sum to get an average
	//to try to reduce the error
	x_axis = (p3 - p0 + p2 - p1);
	x_axis.normalize();
	z_axis = (p1 - p0 + p2 - p3);
	z_axis.normalize();

	y_axis = z_axis ^ x_axis;

	//Since the x_axis and the z_axis were chosen by the user they were
	//not perfectly perpendicular, son recalculate one of them
	z_axis = x_axis ^ y_axis;

	//A matrix which columns are the normalised vectors is
	//the rotation matrix with respect to the origin
	osg::Matrix rot(x_axis.x(), y_axis.x(), z_axis.x(), 0, x_axis.y(),
			y_axis.y(), z_axis.y(), 0, x_axis.z(), y_axis.z(), z_axis.z(), 0, 0,
			0, 0, 1);

	calib_matrix = osg::Matrix::translate(-center) * rot;
}

void CameraCalibrator::save_camera_calibration(int cam_index,
		std::string path) {

	camVecIte cam = camera_arr.begin() + cam_index;
	std::string T_fn(
			path + "/" + (*cam)->get_cam_name() + "/Calibration/T_rgb.txt");

	std::ofstream T_file;
	T_file.precision(20);
	T_file.open(T_fn.c_str());

	//Translate camera using calib_matrix
	osg::Matrix aux_matrix = (*cam)->get_T_osg() * calib_matrix;

	//The matrix is to be saved transposed because when it is read
	//it will be transposed again
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

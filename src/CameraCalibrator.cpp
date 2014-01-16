/*
 * CameraCalibrator.cpp
 *
 *  Created on: 8 Jan 2014
 *      Author: m04701
 */

#include "CameraCalibrator.h"

using std::cout;
using std::endl;

CameraCalibrator::CameraCalibrator(const camVecT& camera_arr_) :
			camera_arr(camera_arr) {
}

CameraCalibrator::~CameraCalibrator() {

}

void CameraCalibrator::set_plate_points(const osg::Vec3& p0_,
		const osg::Vec3& p1_, const osg::Vec3& p2_, const osg::Vec3& p3_) {

	//Use the user points to calculate new origin coordinates
	p0 = p0_;
	p1 = p1_;
	p2 = p2_;
	p3 = p3_;
}

void CameraCalibrator::recalibrate_center_all_cameras() {
	//First calculate translation, for that calculate the centre of the
	//points
	osg::Vec3 center;
	center.x() = (p0.x() + p1.x() + p2.x() + p3.x()) / 4.0;
	center.y() = (p0.y() + p1.y() + p2.y() + p3.y()) / 4.0;
	center.z() = (p0.z() + p1.z() + p2.z() + p3.z()) / 4.0;

	calib_matrix = osg::Matrix::translate(-center);
}

void CameraCalibrator::recalibrate_axes_camera() {

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
	calib_matrix.set(x_axis.x(), y_axis.x(), z_axis.x(), 0, x_axis.y(),
			y_axis.y(), z_axis.y(), 0, x_axis.z(), y_axis.z(), z_axis.z(), 0, 0,
			0, 0, 1);
}

void CameraCalibrator::manual_axes_rotation(float angle, int axes) {
	osg::Vec3 axe_vector;
	switch (axes) {
	case 0:
		axe_vector.set(1.0, 0.0, 0.0);
		break;
	case 1:
		axe_vector.set(0.0, 1.0, 0.0);
		break;
	default:
		axe_vector.set(0.0, 0.0, 1.0);
	}
	calib_matrix = osg::Matrix::rotate(angle, axe_vector);

	constCamVecIte cam = camera_arr.begin();
	for (; cam != camera_arr.end(); ++cam) {
		(*cam)->set_T((*cam)->get_T_osg() * calib_matrix);
	}
}

void CameraCalibrator::save_camera_axes_calibration(int cam_index,
		std::string path) {

	constCamVecIte cam = camera_arr.begin() + cam_index;
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

void CameraCalibrator::save_all_cameras(std::string path) {
	for (unsigned int i = 0; i < camera_arr.size(); i++) {
		save_camera_axes_calibration(i, path);
	}
}

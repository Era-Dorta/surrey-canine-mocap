/*
 * CameraCalibrator.h
 *
 *  Created on: 8 Jan 2014
 *      Author: m04701
 */

#ifndef CAMERACALIBRATOR_H_
#define CAMERACALIBRATOR_H_

#include <osg/Vec3>
#include <osg/Matrix>

#include "RGBDCamera.h"
#include <fstream>

class CameraCalibrator {
	public:
		//TODO Should not give camera_arr as argument
		//In save_camera_calibration, could give camera iterator or
		//only path and matrix
		CameraCalibrator();
		CameraCalibrator(camVecT camera_arr_);
		virtual ~CameraCalibrator();
		void init(camVecT camera_arr_);

		//Point order should be
		//   p1____ p2
		//     |   |
		//     |   |
		//	 p0|___|p3
		void set_plate_points(const osg::Vec3& p0_, const osg::Vec3& p1_,
				const osg::Vec3& p2_, const osg::Vec3& p3_);

		//If either method is called a second time or one call after the other
		//then the previous calibration is lost unless save was used
		void recalibrate_center_all_cameras();
		void recalibrate_axis_camera();

		void save_camera_axis_calibration(int cam_index, std::string path);
		void save_all_cameras_center(std::string path);

	private:
		osg::Vec3 p0, p1, p2, p3;
		osg::Matrix calib_matrix;

		camVecT camera_arr;
};

#endif /* CAMERACALIBRATOR_H_ */

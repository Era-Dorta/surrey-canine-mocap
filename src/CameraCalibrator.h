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
		void set_plate_points(const osg::Vec3& p0, const osg::Vec3& p1,
				const osg::Vec3& p2, const osg::Vec3& p3);

		void save_camera_calibration(int cam_index, std::string path);

	private:
		osg::Matrix calib_matrix;

		camVecT camera_arr;
};

#endif /* CAMERACALIBRATOR_H_ */

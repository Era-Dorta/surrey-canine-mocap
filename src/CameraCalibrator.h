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
	//In save_camera_calibration, could give camera iterator or
	//only path and matrix
	CameraCalibrator(const camVecT& camera_arr_);
	virtual ~CameraCalibrator();

	//Point order should be
	//   p1____ p2
	//     |   |
	//     |   |
	//	 p0|___|p3
	void set_plate_points(const osg::Vec3& p0_, const osg::Vec3& p1_,
			const osg::Vec3& p2_, const osg::Vec3& p3_);

	//If either method is called a second time or one call after the other
	//then the previous calibration is lost unless save was used

	//Used to move the world coordinates to a given point
	void recalibrate_center_all_cameras();

	//Used to change axes orientation using given points
	//It assumes points are not perfectly align and correct its giving
	//preference to X axes
	void recalibrate_axes_camera();

	// X, Y, Z (0, 1, 2)
	void manual_axes_rotation(float angle, int axes);

	//Save calibration for a single camera, useful for fixing missalignments
	//between the cameras, though it does not give good results
	//Better do not use it
	void save_camera_axes_calibration(int cam_index, std::string path);

	//Save calibration for all the cameras
	void save_all_cameras(std::string path);

private:
	osg::Vec3 p0, p1, p2, p3;
	osg::Matrix calib_matrix;

	const camVecT& camera_arr;
};

#endif /* CAMERACALIBRATOR_H_ */

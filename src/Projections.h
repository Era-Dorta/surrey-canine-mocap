/*
 * Projections.h
 *
 *  Created on: 23 Jan 2014
 *      Author: m04701
 */

#ifndef PROJECTIONS_H_
#define PROJECTIONS_H_

#include "CudaVec.h"
#include "RGBDCamera.h"
#include <osg/Vec3>

namespace Projections {
	float3 get_2d_projection(osg::Vec3 point, const osg::Vec3& trans,
			float& z_val);

	float3 get_2d_projection(osg::Vec3 point, const osg::Vec3& trans);

	float4 get_3d_projection(const float3& point, const float& depth,
			const osg::Vec3& trans);

	float3 get_2d_projection(osg::Vec3 point, const float4x4& invT,
			float& z_val);

	float3 get_2d_projection(osg::Vec3 point, const float4x4& invT);

	float4 get_3d_projection(const float3& point, const float& depth,
			const float4x4& T);

	float3 get_2d_projection(osg::Vec3 point, constCamVecIte cam);

	float3 get_3d_projection(int row, int col, constCamVecIte cam, int frame);

	extern float3x3 K;
	extern float3x3 invK;
} /* namespace Projections */
#endif /* PROJECTIONS_H_ */

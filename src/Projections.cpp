/*
 * Projections.cpp
 *
 *  Created on: 23 Jan 2014
 *      Author: m04701
 */

#include "Projections.h"

float3x3 Projections::K;
float3x3 Projections::invK;

float3 Projections::get_2d_projection(osg::Vec3 point, const osg::Vec3& trans,
		float& z_val) {

	//We want a front view so in world axes is vectors
	//x = [0,0,1]
	//y = [0,1,0]
	//z = [-1,0,0]
	//Then also add a translation, matrix should be T*R, but since R is simple
	//it is easier to put translation in new world coordinates
	float4x4 invT(0.0);
	invT[0 * 4 + 2] = -1;
	invT[1 * 4 + 1] = 1;
	invT[2 * 4 + 0] = 1;
	invT[3 * 4 + 0] = trans.z() + 0.05;
	invT[3 * 4 + 1] = trans.y() - 0.15;
	invT[3 * 4 + 2] = -(trans.x() - 0.3);
	invT[3 * 4 + 3] = 1;

	return get_2d_projection(point, invT, z_val);
}

float3 Projections::get_2d_projection(osg::Vec3 point, const osg::Vec3& trans) {
	float zval;
	return get_2d_projection(point, trans, zval);
}

float4 Projections::get_3d_projection(const float3& point, const float& depth,
		const osg::Vec3& trans) {

	float4x4 T(0.0);
	T[0 * 4 + 2] = 1;
	T[1 * 4 + 1] = 1;
	T[2 * 4 + 0] = -1;
	T[3 * 4 + 0] = -(trans.x() - 0.3);
	T[3 * 4 + 1] = -(trans.y() - 0.15);
	T[3 * 4 + 2] = -(trans.z() + 0.05);
	T[3 * 4 + 3] = 1;

	return get_3d_projection(point, depth, T);
}

float3 Projections::get_2d_projection(osg::Vec3 point, const float4x4& invT,
		float& z_val) {
	float4 res4, point4 = make_float4(point.x(), point.y(), point.z(), 1.0);

	res4 = point4 * invT;

	float3 point3 = make_float3(res4.x, res4.y, res4.z);

	float3 res3 = K * point3;

	z_val = res3.z;
	float invz = 1.0 / res3.z;
	res3 = res3 * invz;
	return res3;
}

float3 Projections::get_2d_projection(osg::Vec3 point, const float4x4& invT) {
	float zval;
	return get_2d_projection(point, invT, zval);
}

float4 Projections::get_3d_projection(const float3& point, const float& depth,
		const float4x4& T) {

	float3 depth_pix_hom = make_float3(point.x, point.y, 1.f);

	float3 vert = depth * (invK * depth_pix_hom);

	float4 vert_hom = make_float4(vert, 1.f);

	float4 vert_global = vert_hom * T;
	return vert_global;
}

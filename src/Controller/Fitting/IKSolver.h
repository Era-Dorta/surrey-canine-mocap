/*
 * IKSolver.h
 *
 *  Created on: 29 Jan 2014
 *      Author: m04701
 */

#ifndef IKSOLVER_H_
#define IKSOLVER_H_

#include "../../Misc/CudaVec.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>

class IKSolver {
public:
	IKSolver();

	void start_chain();

	void start_chain(const float3& offset, const float4& rot);

	void start_chain(const float4x4& matrix);

	//rot is a quaternion
	void add_bone_to_chain(const float3& length, const float4& rot);

	//rot is an ZYX euler rotation
	void add_bone_to_chain(const float3& length, const float3& rot);

	bool solve_chain(const float3& goal_position);

	bool solve_chain(const float3& goal_position, const float4& rot);

	unsigned int get_num_joints() const;

	//rot is a quaternion
	void get_rotation_joint(unsigned int index, float4& rot);

	//rot is an ZYX euler rotation
	void get_rotation_joint(unsigned int index, float3& rot);
private:
	bool solve_chain_1_segment(const KDL::Frame& goal, float accuracy = 1e-4,
			unsigned int max_ite = 100);

	bool solve_chain_several_segments(const KDL::Frame& goal, float accuracy =
			1e-4, unsigned int max_ite = 500);

	void transform_rotation_to_kdl(KDL::Rotation& rot);

	void transform_rotation_to_osg(KDL::Rotation& rot);

	void vec_to_kdl(KDL::Vector& vec);

	void vec_to_osg(KDL::Vector& vec);

	void add_angles_to_array(const float3& angles);

	KDL::Chain chain;
	KDL::JntArray solved_joints;
	KDL::JntArray current_joints;
	unsigned int num_joints;
	bool need_extra_joints;
	int extra_segment;
	Eigen::Matrix<double, 6, 1> L;
	KDL::Rotation to_kdl;
	KDL::Rotation to_osg;
};

#endif /* IKSOLVER_H_ */

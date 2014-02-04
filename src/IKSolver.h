/*
 * IKSolver.h
 *
 *  Created on: 29 Jan 2014
 *      Author: m04701
 */

#ifndef IKSOLVER_H_
#define IKSOLVER_H_

#include "CudaVec.h"

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

		void add_bone_to_chain(const float3& length, const float4& rot);

		bool solve_chain(const float3& goal_position,
				unsigned int max_ite = 200, float accuracy = 1e-5);

		unsigned int get_num_joints() const;

		void get_rotation_joint(unsigned int index, float4& rot);
	private:
		KDL::Chain chain;
		KDL::JntArray solved_joints;
		KDL::JntArray current_joints;
		unsigned int num_joints;
		bool need_extra_joints;
		int extra_segment;
		Eigen::Matrix<double, 6, 1> L;
};

#endif /* IKSOLVER_H_ */

/*
 * IKSolver.cpp
 *
 *  Created on: 29 Jan 2014
 *      Author: m04701
 */

#include "IKSolver.h"
#include "DebugUtil.h"

IKSolver::IKSolver() {
	num_joints = 0;
	need_extra_joints = true;
	extra_segment = 0;
	L(0) = 1;
	L(1) = 1;
	L(2) = 1;
	L(3) = 1;
	L(4) = 1;
	L(5) = 1;
}

void IKSolver::start_chain() {
	chain = KDL::Chain();
	current_joints = KDL::JntArray();
	num_joints = 0;
	need_extra_joints = true;
	extra_segment = 0;
}

void IKSolver::start_chain(const float3& offset, const float4& rot) {
	start_chain();

	//Adds an offset to the chain start, since it does not have
	//joints it cannot move
	KDL::Vector kdl_offset(offset.x, offset.y, offset.z);
	KDL::Frame frame(KDL::Rotation::Quaternion(rot.x, rot.y, rot.z, rot.w),
			kdl_offset);

	KDL::Segment segment(KDL::Joint(KDL::Joint::None), frame);
	chain.addSegment(segment);
	extra_segment = 1;
}

void IKSolver::start_chain(const float4x4& matrix) {
	start_chain();

	KDL::Frame frame;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			frame.M(i, j) = matrix.get(i, j);
		}
		frame.p[i] = matrix.get(3, i);
	}

	KDL::Segment segment(KDL::Joint(KDL::Joint::None), frame);
	chain.addSegment(segment);
	extra_segment = 1;
}

void IKSolver::add_bone_to_chain(const float3& length, const float4& rot) {
	//Create a 3DOF joint
	KDL::Joint kdl_jointx(KDL::Joint::RotZ);
	KDL::Joint kdl_jointy(KDL::Joint::RotY);
	KDL::Joint kdl_jointz(KDL::Joint::RotX);

	//Set bone length
	KDL::Vector kdl_length(length.x, length.y, length.z);

	//In KDL a 3DOF joint are two 1DOF joints without length
	//and a third one with with the bone length
	KDL::Segment segmentx(kdl_jointx);
	KDL::Segment segmenty(kdl_jointy);
	KDL::Segment segmentz(kdl_jointz, KDL::Frame(kdl_length));

	//Put the segment in the chain
	chain.addSegment(segmentx);
	chain.addSegment(segmenty);
	chain.addSegment(segmentz);

	int index = current_joints.rows();
	current_joints.resize(current_joints.rows() + 3);

	KDL::Rotation rot_i = KDL::Rotation::Quaternion(rot.x, rot.y, rot.z, rot.w);
	rot_i.GetEulerZYX(current_joints(index), current_joints(index + 1),
			current_joints(index + 2));

	//For our user only one segment was added
	num_joints++;
}

bool IKSolver::solve_chain(const float3& goal_position, unsigned int max_ite,
		float accuracy) {
	if (need_extra_joints) {
		//To be able to solve without aiming we introduce a new virtual joint
		//to cancel the rotations introduced by the chain
		KDL::Segment extra_joint1(KDL::Joint(KDL::Joint::RotZ));
		KDL::Segment extra_joint2(KDL::Joint(KDL::Joint::RotY));
		KDL::Segment extra_joint3(KDL::Joint(KDL::Joint::RotX));

		chain.addSegment(extra_joint1);
		chain.addSegment(extra_joint2);
		chain.addSegment(extra_joint3);

		current_joints.resize(current_joints.rows() + 3);
		need_extra_joints = false;
	}

	//Documentation solver
	//Forward position solver
	//KDL::ChainFkSolverPos_recursive fksolver1(chain);
	//Inverse velocity solver
	//KDL::ChainIkSolverVel_pinv iksolver1v(chain);
	//Inverse position solver with velocity
	//KDL::ChainIkSolverPos_NR iksolver1(chain, fksolver1, iksolver1v, max_ite,
	//		accuracy);

	//According to source code this solver is faster and more accurate,
	//but it is not on the  documentation.
	//It uses a L matrix to weight the rotations and translations
	//But default one fails, so we use our own
	KDL::ChainIkSolverPos_LMA iksolver1(chain, L, accuracy, max_ite);

	//Creation of result array
	solved_joints = KDL::JntArray(chain.getNrOfJoints());

	//Destination frame has identity matrix for rotation
	// and goal position for translation
	KDL::Frame F_dest(
			KDL::Vector(goal_position.x, goal_position.y, goal_position.z));

	//Call the solver
	int exit_flag = iksolver1.CartToJnt(current_joints, F_dest, solved_joints);

	//On success update the current position, this helps the solver to converge
	//faster and produces fluid movements when moving bones manually
	if (exit_flag >= 0) {
		current_joints = solved_joints;
	} else {
		//Manually check how far is from the result, because sometimes
		//specially with only one bone, the iksolver will return -3
		//but the chain will be in a valid position

		//Forward position solver
		KDL::ChainFkSolverPos_recursive fksolver1(chain);

		KDL::Frame solved_pos;
		fksolver1.JntToCart(solved_joints, solved_pos);

		if (KDL::Equal(solved_pos.p, F_dest.p, accuracy)) {
			current_joints = solved_joints;
			return true;
		} else {
			return false;
		}
	}
	return exit_flag >= 0;
}

unsigned int IKSolver::get_num_joints() const {
	return num_joints;
}

void IKSolver::get_rotation_joint(unsigned int index, float4& rot) {
	double x, y, z, w;

	//Since we have three joints in the chain for every one the user added
	index = 3 * index;

	//Rotations angles in every axes
	double angle_z = solved_joints(index);
	double angle_y = solved_joints(index + 1);
	double angle_x = solved_joints(index + 2);

	//If there is an extra offset segment then do not use it for the angles
	index += extra_segment;

	//The total rotation is the product of all the single axes rotations
	(chain.getSegment(index).getJoint().pose(angle_z).M
			* chain.getSegment(index + 1).getJoint().pose(angle_y).M
			* chain.getSegment(index + 2).getJoint().pose(angle_x).M).GetQuaternion(
			x, y, z, w);
	rot.x = x;
	rot.y = y;
	rot.z = z;
	rot.w = w;
}

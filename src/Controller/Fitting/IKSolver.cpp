/*
 * IKSolver.cpp
 *
 *  Created on: 29 Jan 2014
 *      Author: m04701
 */

#include "IKSolver.h"

IKSolver::IKSolver() :
		to_kdl(1, 0, 0, 0, 0, -1, 0, 1, 0), to_osg(1, 0, 0, 0, 0, 1, 0, -1, 0) {
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

	//Adds an offset to the chain, since it does not have
	//joints this segment cannot move
	KDL::Vector kdl_offset(offset.x, offset.y, offset.z);
	vec_to_kdl(kdl_offset);

	KDL::Rotation kdl_rot = KDL::Rotation::Quaternion(rot.x, rot.y, rot.z,
			rot.w);
	transform_rotation_to_kdl(kdl_rot);

	KDL::Frame frame(kdl_rot, kdl_offset);

	KDL::Segment segment(KDL::Joint(KDL::Joint::None), frame);
	chain.addSegment(segment);
	extra_segment = 1;
}

void IKSolver::start_chain(const float4x4& matrix) {
	start_chain();

	KDL::Vector kdl_offset;
	KDL::Rotation kdl_rot;

	//Start the chain with a given transformation matrix
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			kdl_rot(i, j) = matrix.get(i, j);
		}
		kdl_offset[i] = matrix.get(3, i);
	}

	transform_rotation_to_kdl(kdl_rot);
	vec_to_kdl(kdl_offset);

	KDL::Frame frame(kdl_rot, kdl_offset);
	KDL::Segment segment(KDL::Joint(KDL::Joint::None), frame);
	chain.addSegment(segment);
	extra_segment = 1;
}

void IKSolver::add_bone_to_chain(const float3& length, const float4& rot) {

	double x, y, z;
	KDL::Rotation bone_rot = KDL::Rotation::Quaternion(rot.x, rot.y, rot.z,
			rot.w);

	transform_rotation_to_kdl(bone_rot);

	bone_rot.GetEulerZYX(z, y, x);

	add_bone_to_chain(length, make_float3(z, y, x));
}

void IKSolver::add_bone_to_chain(const float3& length, const float3& rot) {
	//Create a 3DOF joint
	KDL::Joint kdl_jointx(KDL::Joint::RotZ);
	KDL::Joint kdl_jointy(KDL::Joint::RotY);
	KDL::Joint kdl_jointz(KDL::Joint::RotX);

	//Set bone length
	KDL::Vector kdl_length(length.x, length.y, length.z);
	vec_to_kdl(kdl_length);

	//In KDL a 3DOF joint are two 1DOF joints without length
	//and a third one with with the bone length
	KDL::Segment segmentx(kdl_jointx);
	KDL::Segment segmenty(kdl_jointy);
	KDL::Segment segmentz(kdl_jointz, KDL::Frame(kdl_length));

	//Put the segment in the chain
	chain.addSegment(segmentx);
	chain.addSegment(segmenty);
	chain.addSegment(segmentz);

	add_angles_to_array(rot);

	//For our user only one segment was added
	num_joints++;
}

bool IKSolver::solve_chain(const float3& goal_position) {
	if (need_extra_joints) {
		//To be able to solve without aiming we introduce a new virtual joint
		//to cancel the rotations introduced by the chain
		KDL::Segment extra_joint1(KDL::Joint(KDL::Joint::RotZ));
		KDL::Segment extra_joint2(KDL::Joint(KDL::Joint::RotY));
		KDL::Segment extra_joint3(KDL::Joint(KDL::Joint::RotX));

		chain.addSegment(extra_joint1);
		chain.addSegment(extra_joint2);
		chain.addSegment(extra_joint3);

		add_angles_to_array(make_float3(0, 0, 0));

		need_extra_joints = false;
	}

	return solve_chain(goal_position, make_float4(0, 0, 0, 1));
}

bool IKSolver::solve_chain(const float3& goal_position, const float4& rot) {
	KDL::Rotation rot_kdl = KDL::Rotation::Quaternion(rot.x, rot.y, rot.z,
			rot.w);
	transform_rotation_to_kdl(rot_kdl);

	KDL::Vector goal_kdl(goal_position.x, goal_position.y, goal_position.z);
	vec_to_kdl(goal_kdl);

	KDL::Frame goal(rot_kdl, goal_kdl);

	if (num_joints == 1) {
		//If we are solving for only one 3DOF joint this method is better
		return solve_chain_1_segment(goal);
	} else {
		//For larger chains this method is better
		return solve_chain_several_segments(goal);
	}
}

unsigned int IKSolver::get_num_joints() const {
	return num_joints;
}

void IKSolver::get_rotation_joint(unsigned int index, float4& rot) {
	double x, y, z, w;

	//Since we have three joints in the chain for every one the user added
	index = 3 * index;

	//Rotations angles in every axis
	double angle_z = solved_joints(index);
	double angle_y = solved_joints(index + 1);
	double angle_x = solved_joints(index + 2);

	//If there is an extra offset segment then do not use it for the angles
	index += extra_segment;

	//The total rotation is the product of all the single axis rotations
	KDL::Rotation quat_rot = (chain.getSegment(index).getJoint().pose(angle_z).M
			* chain.getSegment(index + 1).getJoint().pose(angle_y).M
			* chain.getSegment(index + 2).getJoint().pose(angle_x).M);

	transform_rotation_to_osg(quat_rot);

	quat_rot.GetQuaternion(x, y, z, w);

	rot.x = x;
	rot.y = y;
	rot.z = z;
	rot.w = w;
}

void IKSolver::get_rotation_joint(unsigned int index, float3& rot) {
	//Since we have three joints in the chain for every one the user added
	index = 3 * index;

	//Rotations angles in every axis
	rot.x = solved_joints(index);
	rot.y = solved_joints(index + 1);
	rot.z = solved_joints(index + 2);
}

bool IKSolver::solve_chain_1_segment(const KDL::Frame& goal, float accuracy,
		unsigned int max_ite) {

	//Forward position solver
	KDL::ChainFkSolverPos_recursive fksolver1(chain);
	//Inverse velocity solver
	KDL::ChainIkSolverVel_pinv iksolver1v(chain);
	//Inverse position solver with velocity
	KDL::ChainIkSolverPos_NR iksolver1(chain, fksolver1, iksolver1v, max_ite,
			accuracy);

	//Creation of result array
	solved_joints = KDL::JntArray(chain.getNrOfJoints());

	//Destination frame has identity matrix for rotation
	// and goal position for translation
	//KDL::Frame f_goal(KDL::Rotation::Quaternion(rot.x, rot.y, rot.z, rot.w),
	//		KDL::Vector(goal_position.x, goal_position.y, goal_position.z));

	//Call the solver
	int exit_flag = iksolver1.CartToJnt(current_joints, goal, solved_joints);

	//On success update the current position, this helps the solver to converge
	//faster and produces fluid movements when moving bones manually
	if (exit_flag >= 0) {
		current_joints = solved_joints;
	} else {
		KDL::Frame solved_pos;
		fksolver1.JntToCart(solved_joints, solved_pos);

		//Manually check how far is from the result, because sometimes
		//the iksolver will return -3 but the chain will be in a valid position
		if (KDL::Equal(solved_pos.p, goal.p, accuracy)) {
			current_joints = solved_joints;
			return true;
		} else {
			return false;
		}
	}
	return exit_flag >= 0;
}

bool IKSolver::solve_chain_several_segments(const KDL::Frame& goal,
		float accuracy, unsigned int max_ite) {

	//According to source code this solver is faster and more accurate,
	//but it is not on the  documentation.
	//It uses a L matrix to weight the rotations and translations
	//But default one fails, so we use our own
	KDL::ChainIkSolverPos_LMA iksolver1(chain, L, accuracy, max_ite);

	//Creation of result array
	solved_joints = KDL::JntArray(chain.getNrOfJoints());

	//Call the solver
	int exit_flag = iksolver1.CartToJnt(current_joints, goal, solved_joints);

	//On success update the current position, this helps the solver to converge
	//faster and produces fluid movements when moving bones manually
	if (exit_flag >= 0) {
		current_joints = solved_joints;
	}

	return exit_flag >= 0;
}

void IKSolver::transform_rotation_to_kdl(KDL::Rotation& rot) {
	//Get the rotation in KDL coordinate system
	rot = to_kdl * rot * to_osg;
}

void IKSolver::transform_rotation_to_osg(KDL::Rotation& rot) {
	//Get the rotation in osg coordinate system
	rot = to_osg * rot * to_kdl;
}

void IKSolver::vec_to_kdl(KDL::Vector& vec) {
	vec = to_kdl * vec;
}

void IKSolver::vec_to_osg(KDL::Vector& vec) {
	vec = to_osg * vec;
}

void IKSolver::add_angles_to_array(const float3& angles) {
	//For some reason JntArray.resize() deletes all the previous
	//content even if new size is bigger, so we have to do a
	//manual resize and copy values
	int index = current_joints.rows();
	KDL::JntArray aux_arr(current_joints.rows() + 3);

	for (unsigned int i = 0; i < current_joints.rows(); i++) {
		aux_arr(i) = current_joints(i);
	}

	aux_arr(index) = angles.x;
	aux_arr(index + 1) = angles.y;
	aux_arr(index + 2) = angles.z;

	current_joints = aux_arr;
}

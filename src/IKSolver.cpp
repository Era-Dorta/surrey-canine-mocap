/*
 * IKSolver.cpp
 *
 *  Created on: 29 Jan 2014
 *      Author: m04701
 */

#include "IKSolver.h"

IKSolver::IKSolver() {
	num_joints = 0;
}

void IKSolver::start_chain() {
	chain = KDL::Chain();
	num_joints = 0;
}
void IKSolver::add_bone_to_chain(const float3& offset, const float4& rot) {
	//Create a 3DOF joint
	KDL::Joint kdl_jointx(KDL::Joint::RotX);
	KDL::Joint kdl_jointy(KDL::Joint::RotY);
	KDL::Joint kdl_jointz(KDL::Joint::RotZ);

	//Set bone length
	KDL::Vector kdl_offset(offset.x, offset.y, offset.z);

	//In KDL a 3DOF joint are 2 1DOF joints without length
	//and the last with the length
	KDL::Segment segmentx(kdl_jointx);
	KDL::Segment segmenty(kdl_jointy);
	KDL::Segment segmentz(kdl_jointz, KDL::Frame(kdl_offset));

	//Put the segment in the chain
	chain.addSegment(segmentx);
	chain.addSegment(segmenty);
	chain.addSegment(segmentz);

	//For our user only one segment was added
	num_joints++;
}

bool IKSolver::solve_chain(const float3& goal_position, unsigned int max_ite,
		float accuracy) {
	//To be able to solve without aiming we introduce a new virtual joint to
	//cancel the rotations introduced by the chain
	KDL::Segment extra_seg1(KDL::Joint(KDL::Joint::RotX));
	KDL::Segment extra_seg2(KDL::Joint(KDL::Joint::RotY));
	KDL::Segment extra_seg3(KDL::Joint(KDL::Joint::RotZ));

	chain.addSegment(extra_seg1);
	chain.addSegment(extra_seg2);
	chain.addSegment(extra_seg3);

	//Creation of the solvers:
	KDL::ChainFkSolverPos_recursive fksolver1(chain); //Forward position solver
	KDL::ChainIkSolverVel_pinv iksolver1v(chain); //Inverse velocity solver

	//Maximum 100 iterations, stop at accuracy 1e-6
	KDL::ChainIkSolverPos_NR iksolver1(chain, fksolver1, iksolver1v, max_ite,
			accuracy);

	//Creation of jntarrays:
	solved_joints = KDL::JntArray(chain.getNrOfJoints());
	KDL::JntArray q_init(chain.getNrOfJoints());

	//Destination frame has identity rotation and goal position translation
	KDL::Vector destination(goal_position.x, goal_position.y, goal_position.z);
	KDL::Frame F_dest(destination);

	int resi = iksolver1.CartToJnt(q_init, F_dest, solved_joints);
	return resi >= 0;
}

unsigned int IKSolver::get_num_joints() const {
	return num_joints;
}

void IKSolver::get_rotation_joint(unsigned int index, float4& rot) {
	double x, y, z, w;
	//Since we have 3 joints in the chain for every one the user added
	index = index + 3 * index;

	//Rotations angles in every axes
	double angle_x = solved_joints(index);
	double angle_y = solved_joints(index + 1);
	double angle_z = solved_joints(index + 2);

	//The total rotation is the product of all the single axes rotations
	(chain.getSegment(index).getJoint().pose(angle_x).M
			* chain.getSegment(index + 1).getJoint().pose(angle_y).M
			* chain.getSegment(index + 2).getJoint().pose(angle_z).M).GetQuaternion(
			x, y, z, w);
	rot.x = x;
	rot.y = y;
	rot.z = z;
	rot.w = w;
}

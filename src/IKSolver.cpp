/*
 * IKSolver.cpp
 *
 *  Created on: 29 Jan 2014
 *      Author: m04701
 */

#include "IKSolver.h"

IKSolver::IKSolver() {

}

void IKSolver::start_chain() {
	chain = KDL::Chain();
}
void IKSolver::add_bone_to_chain(const osg::Vec3& offset,
		const osg::Quat& rot) {
	KDL::Joint kdl_joint(KDL::Joint::RotAxis);

	KDL::Rotation kdl_rot;
	KDL::Vector kdl_offset(offset.x(), offset.y(), offset.z());

	KDL::Segment segment(kdl_joint, KDL::Frame(kdl_rot, kdl_offset));

	chain.addSegment(segment);
}

bool IKSolver::solve_chain(const osg::Vec3& goal_position) {
	//Creation of the solvers:
	KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
	KDL::ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver1(chain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	//Creation of jntarrays:
	KDL::JntArray q(chain.getNrOfJoints());
	KDL::JntArray q_init(chain.getNrOfJoints());

	//Set destination frame
	KDL::Frame F_dest;

	int ret = iksolver1.CartToJnt(q_init,F_dest,q);
	return true;
}

unsigned int IKSolver::get_num_segments() const {
	return chain.getNrOfSegments();
}

void IKSolver::get_rotation_joint(int index, osg::Quat& rot) {
}

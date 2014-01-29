/*
 * IKSolver.h
 *
 *  Created on: 29 Jan 2014
 *      Author: m04701
 */

#ifndef IKSOLVER_H_
#define IKSOLVER_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>

class IKSolver {
	public:
		IKSolver();

		void start_chain();

		void add_bone_to_chain(const osg::Vec3& offset,
				const osg::Quat& rot);

		bool solve_chain(const osg::Vec3& goal_position);

		unsigned int get_num_segments() const;

		void get_rotation_joint( int index, osg::Quat& rot);
	private:
		KDL::Chain chain;
};

#endif /* IKSOLVER_H_ */

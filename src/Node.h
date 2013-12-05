/*
 * Node.h
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */

#ifndef NODE_H_
#define NODE_H_

#include <osg/Array>
#include <osg/MatrixTransform>
#include <cstddef>
#include <vector>
#include <string>
#include <cmath>
#include "boost/shared_ptr.hpp"
#include "boost/weak_ptr.hpp"

typedef unsigned char BYTE;

class Node;

typedef boost::shared_ptr<Node> NodePtr;
typedef boost::weak_ptr<Node> wNodePtr;

class Node {
	public:
		Node();
		virtual ~Node();

		void increase_no_children();

		void setup_offset(float x = 0.0f, float y = 0.0f, float z = 0.0f);

		void resize_frame_no(long frames);

		unsigned int get_num_children();

		Node* get_last_child();

		void calculate_quats(osg::ref_ptr<osg::Vec3Array> axis);

		void update_euler_angles();

		void toggle_color();

		osg::Quat get_global_rot(int frame_num);

		osg::Quat get_inv_global_rot(int frame_num);

		osg::Vec3 get_global_pos(int frame_num);

		osg::Vec3 get_end_bone_global_pos(int frame_num);

		void get_global_matrix(int frame_num, osg::Matrix& trans);

		//TODO Right now we use quaternions but it might be a good optimisation
		//to have all the matrices precomputed once there are not going to be
		//more changes
		std::string name;
		osg::Vec3f length;    // length of segment
		osg::Vec3f offset; // Transitional offset with respect to the end of the parent link
		std::vector<NodePtr> children;    // Array of pointers to child nodes
		//TODO Instead of Node* use weak pointers
		Node *parent;       // Back pointer to parent node
		osg::ref_ptr<osg::Vec3Array> froset;  // Array of offsets for each frame
		osg::ref_ptr<osg::Vec3Array> freuler;  // Array of angles for each frame
		std::vector<osg::Quat> quat_arr;
		BYTE DOFs;          // Used to determine what DOFs the segment has
		int noofchannels;

		//MatrixTransform that is drawing this node, this is high coupling
		//but it simplifies the code
		osg::MatrixTransform* osg_node;

		osg::Vec4 n_joint_color;
		osg::Vec4 n_bone_color;

		const static float bone_radius;
		const static float joint_radius;
	private:
		void calculate_world_matrix(Node* node, osg::Matrix& trans,
				int frame_num);

		const static osg::Vec4 joint_color;
		const static osg::Vec4 joint_second_color;
		const static osg::Vec4 bone_color;
		const static osg::Vec4 bone_second_color;

		void quat_to_euler(osg::Quat& q, osg::Vec3& euler);
};
#endif /* NODE_H_ */

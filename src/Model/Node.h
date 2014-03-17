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
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

typedef unsigned char BYTE;

class Node;

typedef boost::shared_ptr<Node> NodePtr;
typedef boost::weak_ptr<Node> wNodePtr;

class Node {
public:
	Node();
	virtual ~Node();

	void increase_no_children();

	void resize_frame_no(long frames);

	unsigned int get_num_children();

	Node* get_last_child();

	void calculate_quats(osg::ref_ptr<osg::Vec3Array> axis);

	void optimize_rotations_all_frames();

	//For this method to work first call calculate_quats
	void optimize_rotation(int n_frame);

	void update_euler_angles();

	void toggle_color();

	osg::Quat get_global_rot(int frame_num);

	osg::Quat get_inv_global_rot(int frame_num);

	osg::Vec3 get_global_pos(int frame_num);

	osg::Vec3 get_end_bone_global_pos(int frame_num);

	void get_global_matrix(int frame_num, osg::Matrix& trans);

	void get_parent_to_bone_end_matrix(int frame_num, osg::Matrix& m);

	void get_node_world_matrix_origin(int frame_num, osg::Matrix& matrix);

	const osg::Vec3& get_x_axis(int n_frame) const;
	void set_x_axis(int n_frame, const osg::Vec3& new_axis);
	const osg::Vec3& get_y_axis(int n_frame) const;
	void set_y_axis(int n_frame, const osg::Vec3& new_axis);
	const osg::Vec3& get_z_axis(int n_frame) const;
	void set_z_axis(int n_frame, const osg::Vec3& new_axis);

	const osg::Vec3f& get_local_end() const;
	void set_local_end(const osg::Vec3f& local_end);

	const osg::Vec3f& get_offset() const;
	void set_offset(const osg::Vec3f& offset);

	float get_length() const;
	float get_length2() const;

	//TODO Right now we use quaternions but it might be a good optimisation
	//to have all the matrices precomputed once there are not going to be
	//more changes
	std::string name;
	std::vector<NodePtr> children; // Array of pointers to child nodes
	Node *parent; // Back pointer to parent node
	osg::ref_ptr<osg::Vec3Array> froset; // Array of offsets for each frame
	osg::ref_ptr<osg::Vec3Array> freuler; // Array of angles for each frame
	std::vector<osg::Quat> quat_arr;
	BYTE DOFs; // Used to determine what DOFs the segment has
	int noofchannels;

	//MatrixTransform that is drawing this node, this is high coupling
	//but it simplifies the code
	osg::MatrixTransform* osg_node;
	//Matrix that points to the rendered axis for this node, more code coupling
	osg::MatrixTransform* osg_axis;

	osg::Vec4 n_joint_color;
	osg::Vec4 n_bone_color;

	const static float bone_radius;
	const static float joint_radius;
	const static osg::Vec4 joint_color;
private:
	void calculate_world_matrix(osg::Matrix& trans, int frame_num);

	bool equivalent(const osg::Vec3& vec0, const osg::Vec3& vec1);

	bool equivalent(const osg::Quat& q0, const osg::Quat& q1);

	void set_rotation_axis(int n_frame);

	void quat_to_euler(osg::Quat& q, osg::Vec3& euler);

	void correct_descendants_axis(int frame_num);

	osg::Vec3f local_end; // length of segment
	osg::Vec3f offset; // Transitional offset with respect to the end of the parent link
	float length;
	float length2;

	const static osg::Vec4 joint_second_color;
	const static osg::Vec4 bone_color;
	const static osg::Vec4 bone_second_color;

	//Custom rotation axis to give a more intuitive
	//user rotations
	osg::ref_ptr<osg::Vec3Array> x_axis;
	osg::ref_ptr<osg::Vec3Array> y_axis;
	osg::ref_ptr<osg::Vec3Array> z_axis;
};
#endif /* NODE_H_ */

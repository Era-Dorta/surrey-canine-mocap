/*
 * Node.cpp
 *
 *  Created on: 13 Nov 2013
 *      Author: m04701
 */
#include "Node.h"

const osg::Vec4 Node::joint_color(0.5f, 0.5f, 0.5f, 1.0); //Grey
const osg::Vec4 Node::joint_second_color(1.0f, 1.0f, 1.0f, 1.0); //White
const osg::Vec4 Node::bone_color(0.0f, 0.0f, 1.0f, 1.0); //Blue
const osg::Vec4 Node::bone_second_color(1.0f, 0.0f, 0.0f, 1.0); //Red

const float Node::bone_radius = 0.005;
const float Node::joint_radius = 0.01;

Node::Node() {
	parent = NULL;
	froset = new osg::Vec3Array();
	freuler = new osg::Vec3Array();
	DOFs = 0;
	noofchannels = 0;
	osg_node = NULL;
	osg_axis = NULL;
	n_joint_color = joint_color;
	n_bone_color = bone_color;
	length = 0;
	length2 = 0;
	x_axis = new osg::Vec3Array();
	y_axis = new osg::Vec3Array();
	z_axis = new osg::Vec3Array();
}

Node::~Node() {
}

void Node::increase_no_children() {
	children.push_back(NodePtr(new Node));
	children.back()->parent = this;
}

void Node::resize_frame_no(long frames) {
	//If the vectors are not empty then resize but fill with the value of the
	//last element
	if (freuler->size() > 0) {
		freuler->resize(frames, freuler->back());
		froset->resize(frames, froset->back());
		quat_arr.resize(frames, quat_arr.back());
		x_axis->resize(frames, x_axis->back());
		y_axis->resize(frames, y_axis->back());
		z_axis->resize(frames, z_axis->back());
	} else {
		freuler->resize(frames);
		froset->resize(frames);
		quat_arr.resize(frames);
		x_axis->resize(frames);
		y_axis->resize(frames);
		z_axis->resize(frames);
	}
}

unsigned int Node::get_num_children() {
	return children.size();
}

Node* Node::get_last_child() {
	return children.back().get();
}

void Node::calculate_quats(osg::ref_ptr<osg::Vec3Array> axis) {
	//To build the quaternion we use an axis and a rotation around that axis
	//from the bvh file. Since OSG uses premultiplication the order of the
	//rotations is build backwards
	for (unsigned int i = 0; i < freuler->size(); i++) {
		quat_arr.at(i) = osg::Quat(freuler->at(i)[2], axis->at(2),
				freuler->at(i)[1], axis->at(1), freuler->at(i)[0], axis->at(0));
	}
}

void Node::optimize_rotations_all_frames() {
	for (unsigned int i = 0; i < quat_arr.size(); i++) {
		optimize_rotation(i);
	}
}

void Node::optimize_rotation(int n_frame) {
	//Important this breaks skeletons that have bones separated from
	//the parent, with empty space in the middle, if that is the case
	//apply this to only bones that have one or no children.

	osg::Vec3 frame_end_pos = quat_arr.at(n_frame) * local_end;
	osg::Quat new_rot;
	//Calculate rotation from local_end without rotation to
	//where local_end should be for this frame,
	//makeRotate gives the fastest and simplest rotation
	new_rot.makeRotate(local_end, frame_end_pos);

	//If the rotation is already the fastest one do not anything
	if (new_rot != quat_arr.at(n_frame)) {
		osg::Quat prev_rot = quat_arr.at(n_frame);
		//New rotation is the fastest one
		quat_arr.at(n_frame) = new_rot;

		//Update children rotation to avoid a change in the skeleton position
		osg::Quat extra_rot = prev_rot * new_rot.inverse();
		for (unsigned int i = 0; i < children.size(); i++) {
			// To calculate new child rotation lets call previous parent rotation
			// Q1, new parent rotation Q1' and lets use index 2 for child rotations
			// So we previous state is describe by:
			// Q2 * Q1 = QT -> Where QT is total rotation at child end position
			// New rotation will be
			// Q2' * Q1' = QT
			// Isolating Q2'
			// Q2' = Q2 * Q1 * inv(Q1')
			children[i]->quat_arr.at(n_frame) = children[i]->quat_arr.at(
					n_frame) * extra_rot;
			correct_descendants_axis(n_frame);
		}
	}
	set_rotation_axis(n_frame);
}

void Node::set_rotation_axis(int n_frame) {
	if (parent == NULL) {
		x_axis->at(n_frame) = quat_arr.at(n_frame) * local_end;
		x_axis->at(n_frame).normalize();
		y_axis->at(n_frame).set(
				x_axis->at(n_frame).y() - x_axis->at(n_frame).z(),
				-x_axis->at(n_frame).x(), x_axis->at(n_frame).x());
		y_axis->at(n_frame).normalize();
		z_axis->at(n_frame) = x_axis->at(n_frame) ^ y_axis->at(n_frame);
		z_axis->at(n_frame).normalize();
		return;
	}

	//x_axis is the vector that goes to local_end position for this frame
	x_axis->at(n_frame) = quat_arr.at(n_frame) * local_end;
	x_axis->at(n_frame).normalize();

	//Calculate the vector normal to this bone and its parent bone
	osg::Vec3 normal_vec = parent->local_end ^ x_axis->at(n_frame);
	normal_vec.normalize();

	//To have a common criteria lets prefer normal vectors with positive z
	osg::Vec3 normal_vec_glob = get_global_rot(n_frame) * normal_vec;
	if (normal_vec_glob.z() < 0) {
		normal_vec = -normal_vec;
	}

	//If the bone dir and parent dir are the parallel then use parent y axis
	if (normal_vec == osg::Vec3(0, 0, 0)) {
		normal_vec = parent->y_axis->at(n_frame);
	}

	y_axis->at(n_frame) = normal_vec;

	// z axis is one normal axis to x and y axis
	z_axis->at(n_frame) = x_axis->at(n_frame) ^ y_axis->at(n_frame);
	z_axis->at(n_frame).normalize();
}

void Node::update_euler_angles() {
	for (unsigned int i = 0; i < quat_arr.size(); i++) {
		quat_to_euler(quat_arr.at(i), freuler->at(i));
	}
}

osg::Quat Node::get_global_rot(int frame_num) {
	osg::Quat res;
	Node * prev_node = this;

	do {
		res = res * prev_node->quat_arr.at(frame_num);
		prev_node = prev_node->parent;
	} while (prev_node != NULL);

	return res;
}

osg::Quat Node::get_inv_global_rot(int frame_num) {
	return get_global_rot(frame_num).inverse();
}

osg::Vec3 Node::get_global_pos(int frame_num) {
	osg::Matrix aux;
	calculate_world_matrix(aux, frame_num);

	return osg::Vec3() * aux;
}

osg::Vec3 Node::get_end_bone_global_pos(int frame_num) {
	osg::Matrix aux;
	calculate_world_matrix(aux, frame_num);
	return local_end * aux;
}

void Node::get_global_matrix(int frame_num, osg::Matrix& trans) {
	calculate_world_matrix(trans, frame_num);
}

void Node::get_parent_to_bone_end_matrix(int frame_num, osg::Matrix& m) {

	m.makeIdentity();

	if (parent == NULL) {
		return;
	}

	//Calculate the transformation from the parent
	//to the end of this bone end position
	m = osg::Matrix::translate(local_end)
			* osg::Matrix::rotate(quat_arr.at(frame_num))
			* osg::Matrix::translate(parent->local_end)
			* osg::Matrix::rotate(parent->quat_arr.at(frame_num));
}

void Node::calculate_world_matrix(osg::Matrix& trans, int frame_num) {
	Node* node = this;
	trans.makeIdentity();
	while (node != NULL) {
		trans = trans * osg::Matrix::rotate(node->quat_arr.at(frame_num))
				* osg::Matrix::translate(
						node->offset + node->froset->at(frame_num));
		node = node->parent;
	}
}

void Node::quat_to_euler(osg::Quat& q, osg::Vec3& euler) {
	//To calculate back the euler angles from a quaternion we use this formulas
	//but since we want the postmultiplication order we use the conjugate of the
	//quaternion, also there has to be a change in the sign of the angles
	//http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	osg::Quat q_c = q.conj();
	euler[0] = -std::atan2(2.0 * (q_c.y() * q_c.z() + q_c.w() * q_c.x()),
			q_c.w() * q_c.w() - q_c.x() * q_c.x() - q_c.y() * q_c.y()
					+ q_c.z() * q_c.z());
	euler[1] = -std::asin(-2.0 * (q_c.x() * q_c.z() - q_c.w() * q_c.y()));
	euler[2] = -std::atan2(2.0 * (q_c.x() * q_c.y() + q_c.w() * q_c.z()),
			q_c.w() * q_c.w() + q_c.x() * q_c.x() - q_c.y() * q_c.y()
					- q_c.z() * q_c.z());
}

void Node::toggle_color() {
	if (n_joint_color != joint_color) {
		n_joint_color = joint_color;
		n_bone_color = bone_color;
	} else {
		n_joint_color = joint_second_color;
		n_bone_color = bone_second_color;
	}
}

void Node::get_node_world_matrix_origin(int frame_num, osg::Matrix& matrix) {

	if (parent) {
		parent->get_global_matrix(frame_num, matrix);
	}

	matrix = osg::Matrix::translate(offset + froset->at(frame_num)) * matrix;
	matrix = osg::Matrix::inverse(matrix);
}

bool Node::equivalent(const osg::Vec3& vec0, const osg::Vec3& vec1) {
	return osg::equivalent(vec0.x(), vec1.x(), (float) 1e-4)
			&& osg::equivalent(vec0.y(), vec1.y(), (float) 1e-4)
			&& osg::equivalent(vec0.z(), vec1.z(), (float) 1e-4);
}

bool Node::equivalent(const osg::Quat& q0, const osg::Quat& q1) {
	return osg::equivalent(q0.x(), q1.x(), 1e-4)
			&& osg::equivalent(q0.y(), q1.y(), 1e-4)
			&& osg::equivalent(q0.z(), q1.z(), 1e-4)
			&& osg::equivalent(q0.w(), q1.w(), 1e-4);
}

void Node::correct_descendants_axis(int frame_num) {
	set_rotation_axis(frame_num);
	for (unsigned int i = 0; i < children.size(); i++) {
		children[i]->correct_descendants_axis(frame_num);
	}
}

const osg::Vec3& Node::get_x_axis(int n_frame) const {
	return x_axis->at(n_frame);
}

void Node::set_x_axis(int n_frame, const osg::Vec3& new_axis) {
	x_axis->at(n_frame) = new_axis;
}

const osg::Vec3& Node::get_y_axis(int n_frame) const {
	return y_axis->at(n_frame);
}

void Node::set_y_axis(int n_frame, const osg::Vec3& new_axis) {
	y_axis->at(n_frame) = new_axis;
}

const osg::Vec3& Node::get_z_axis(int n_frame) const {
	return z_axis->at(n_frame);
}

void Node::set_z_axis(int n_frame, const osg::Vec3& new_axis) {
	z_axis->at(n_frame) = new_axis;
}

const osg::Vec3f& Node::get_local_end() const {
	return local_end;
}

void Node::set_local_end(const osg::Vec3f& local_end) {
	this->local_end = local_end;
	length = local_end.length();
	length2 = length * length;
}

const osg::Vec3f& Node::get_offset() const {
	return offset;
}

void Node::set_offset(const osg::Vec3f& offset) {
	this->offset = offset;
	length = local_end.length();
	length2 = length * length;
}

float Node::get_length() const {
	return length;
}

float Node::get_length2() const {
	return length2;
}

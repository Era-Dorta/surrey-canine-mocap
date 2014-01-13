/*
 * SkeletonFitting.cpp
 *
 *  Created on: 5 Nov 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"
#include "DebugUtil.h"

bool comp_x(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.x() < j.x());
}

bool comp_y(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.y() < j.y());
}

bool comp_z(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.z() < j.z());
}

SkeletonFitting::SkeletonFitting() :
			move_joint_max_dist(0), error_threshold(0.005), current_frame(-1) {
}

SkeletonFitting::~SkeletonFitting() {
}

void SkeletonFitting::init(boost::shared_ptr<Skeleton> skeleton_,
		boost::shared_ptr<Skeletonization3D> skeletonization3d) {
	skeleton = skeleton_;
	skeletonizator = skeletonization3d;
}

//void SkeletonFitting::fit_skeleton_into_cloud(Skeleton& skeleton,
//		osg::ref_ptr<osg::Vec3Array> cloud) {

//TODO
//Select all points that are closer than a threshold to the line formed by
//a pair of joints, care with points far away from the joints, case the line
//cuts other points in the skeleton->

//How to get the points???
//Segment in boxes first???

//Call fitLine with those points and the line formed by the joints
//New joints shall be the closest points of the new line to the previous
//joints

//fitLine(InputArray points, OutputArray line, int distType, double param, double reps, double aeps)
//points – Input vector of 2D or 3D points, stored in std::vector<> or Mat.
//line – Output line parameters. In case of 2D fitting, it should be a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0), where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
//distType – Distance used by the M-estimator (see the discussion below).
//param – Numerical parameter ( C ) for some types of distances. If it is 0, an optimal value is chosen.
//reps – Sufficient accuracy for the radius (distance between the coordinate origin and the line).
//aeps – Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.

//}

//void SkeletonFitting::fit_skeleton_with_prev_nex_frame(Skeleton& skeleton,
//	int frame) {
//TODO
//Calculate distance to joint in next and previous frame
//Calculate vectors of movement to positions in next and previous frame
//Add them
//If the are not 0 in any dimension, it means that if we move in that dimension
//a certain amount, then distance to prev and next should be reduced.
//Iterative??? How much to move???
//}

void SkeletonFitting::calculate_for_frame(int frame_num) {
	if (current_frame != frame_num) {
		current_frame = frame_num;
		cloud = skeletonizator->get_merged_3d_projection(current_frame);
		divide_four_sections();
	}
}

void SkeletonFitting::fit_root_position() {
	int head_index = find_head();

	if (head_index != -1) {
		osg::Vec3 translation = cloud->at(head_index)
				- skeleton->get_root()->offset
				- skeleton->get_root()->froset->at(current_frame);
		skeleton->translate_root(translation);
	}
}

void SkeletonFitting::fit_leg_position(Skel_Leg leg) {
	std::vector<int> leg_points_index, joint_positions_index;

	//TODO find_paw initialises leg_points_index, should have another method
	//to do this

	int paw_index = find_paw(leg, leg_points_index);

	if (paw_index != -1) {
		//TODO Not sure if this should be here or in some other place
		//Since we are going to go up the leg better to have all the points ordered
		//along the y axis
		sortstruct s(this);
		std::sort(leg_points_index.begin(), leg_points_index.end(), s);

		int bones_per_leg = 4;
		joint_positions_index.resize(bones_per_leg);

		joint_positions_index[0] = paw_index;
		//TODO Much more efficient to have the iterate backwards

		std::vector<int>::iterator j = leg_points_index.begin();
		for (int i = 1; i < bones_per_leg; i++) {

			float bone_length =
					skeleton->get_node(leg - i + 1)->length.length();
			//Set bone length to paw_index
			//Go up bone length
			bool not_bone_length = true;

			while (not_bone_length && j != leg_points_index.end()) {

				float current_length = (cloud->at(joint_positions_index[i - 1])
						- cloud->at(*j)).length();
				if (current_length >= bone_length) {
					not_bone_length = false;
				} else {
					j++;
				}
			}

			joint_positions_index[i] = *j;
		}

		//Solve for "shoulder" two bones
		solve_2_bones(leg - 3, leg - 2, cloud->at(joint_positions_index[2]));

		//Solve for paw and parent bone
		solve_2_bones(leg - 1, leg, cloud->at(joint_positions_index[0]));
	}
}

const std::vector<Skel_Leg>& SkeletonFitting::getLabels() const {
	return labels;
}

osg::Vec3 SkeletonFitting::get_paw(Skel_Leg leg) {
	std::vector<int> leg_points_index;
	int index = find_paw(leg, leg_points_index);

	if (index != -1) {
		return cloud->at(index);
	} else {
		return osg::Vec3();
	}
}

int SkeletonFitting::find_head() {
	int index = -1;

	if (cloud->size() > 4) {
		divide_four_sections(cloud);
		float max_x = cloud->front().x();
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] == Not_Limbs && max_x < cloud->at(i).x()) {
				max_x = cloud->at(i).x();
				index = i;
			}
		}
	}
	return index;
}

int SkeletonFitting::find_paw(Skel_Leg leg,
		std::vector<int>& leg_points_index) {
	leg_points_index.clear();

	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (labels[i] == leg) {
			leg_points_index.push_back(i);
		}
	}

	if (leg_points_index.size() > 0) {
		float max_y = cloud->at(leg_points_index.front()).y();
		int index = leg_points_index.front();
		for (unsigned int i = 0; i < leg_points_index.size(); i++) {
			if (max_y < cloud->at(leg_points_index[i]).y()) {
				max_y = cloud->at(leg_points_index[i]).y();
				index = leg_points_index[i];
			}
		}
		return index;
	} else {
		return -1;
	}
}

void SkeletonFitting::divide_four_sections(bool use_median) {
	labels.clear();
	labels.resize(cloud->size(), Front_Left);

	if (cloud->size() >= 4) {
		float mean_y;
		if (use_median) {
			mean_y = get_median(cloud, Front_Left, Y);
		} else {
			mean_y = get_mean(cloud, Front_Left, Y);
		}
		int num_invalid = 0;
		//Divide in half vertically, discard all values above
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (cloud->at(i).y() < mean_y) {
				labels[i] = Not_Limbs;
				num_invalid++;
			}
		}

		//Divide the remaining values in front/back part along x
		float mean_x;
		if (use_median) {
			mean_x = get_median(cloud, Front_Left, X);
		} else {
			mean_x = get_mean(cloud, Front_Left, X);
		}
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] == Front_Left && cloud->at(i).x() <= mean_x) {
				labels[i] = Back_Left;
			}
		}

		//Divide the two groups into left and right
		float mean_z_front;
		float mean_z_back;
		if (use_median) {
			mean_z_front = get_median(cloud, Front_Left, Z);
			mean_z_back = get_median(cloud, Back_Left, Z);
		} else {
			mean_z_front = get_mean(cloud, Front_Left, Z);
			mean_z_back = get_mean(cloud, Back_Left, Z);
		}
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] == Front_Left && cloud->at(i).z() < mean_z_front) {
				labels[i] = Front_Right;
			} else if (labels[i] == Back_Left
					&& cloud->at(i).z() < mean_z_back) {
				labels[i] = Back_Right;
			}
		}

	}

	/*//Kmeans does not gives good results, but leave code here in case it would
	 // be used later
	 int num_valid = cloud->size() - num_invalid;
	 int max_clusters = 4;
	 cv::Mat labels;

	 cv::Mat data2(num_valid, 1, CV_32FC3);
	 for(int i = 0; i < num_valid; i++) {
	 if(result[i] != Not_Use){
	 cv::Point3f ipt(cloud->at(i).x(),
	 cloud->at(i).y(), cloud->at(i).z() );
	 data2.at<cv::Point3f>(i) =ipt;
	 }
	 }

	 cv::kmeans(data2, max_clusters, labels,
	 cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3,
	 cv::KMEANS_PP_CENTERS);

	 int j = 0;
	 for(int i = 0; i < num_valid; i++) {
	 if(result[i] != Not_Limbs){
	 result[i] = (Skel_Leg)labels.at<int>(j);
	 j++;
	 }
	 }*/
}

bool SkeletonFitting::solve_2_bones(int bone0, int bone1,
		const osg::Vec3& position, float swivel_angle) {
	//TODO Calculate axis according to bones, this is arbitrary and
	//it should not always work

	//Positive direction axis, axis pointing out of the body
	const float Xaxis[] = { 1, 0, 0 };
	//Projection axis, used to determine one of the axis of the local
	//coordinate system
	const float Yaxis[] = { 0, 1, 0 };

	if (!check_bone_index(bone0, bone1)) {
		cout << "solve_2_bones invalid bone index" << endl;
		return false;
	}

	osg::Matrix bone_world_matrix_off;
	Node* n_bone_0, *n_bone_1;
	n_bone_0 = skeleton->get_node(bone0);
	n_bone_1 = skeleton->get_node(bone1);
	if (n_bone_0->parent) {
		n_bone_0->parent->get_global_matrix(current_frame,
				bone_world_matrix_off);
	}

	bone_world_matrix_off = osg::Matrix::translate(
			n_bone_0->offset + n_bone_0->froset->at(current_frame))
			* bone_world_matrix_off;
	bone_world_matrix_off = osg::Matrix::inverse(bone_world_matrix_off);

	osg::Quat prev_rot_0, prev_rot_1;
	prev_rot_0 = n_bone_0->quat_arr.at(current_frame);
	prev_rot_1 = n_bone_1->quat_arr.at(current_frame);

	Matrix T, S;

	osg_to_matrix(T, osg::Matrix::translate(n_bone_0->length));
	osg_to_matrix(S, osg::Matrix::translate(n_bone_1->length));

	SRS s(T, S, Yaxis, Xaxis);

	Matrix R1, G;
	osg_to_matrix(G, osg::Matrix::translate(position * bone_world_matrix_off));

	float eangle = 0.0;
	if (s.SetGoal(G, eangle)) {
		s.SolveR1(swivel_angle, R1);

		osg::Matrix osg_mat;
		osg::Quat q;

		matrix_to_osg(osg_mat, R1);
		q.set(osg_mat);

		n_bone_0->quat_arr.at(current_frame) = q;

		q = osg::Quat(eangle, osg::Vec3(0.0, 1.0, 0.0));
		n_bone_1->quat_arr.at(current_frame) = q;
	} else {
		cout << "Can not put joint on coordinates eangle " << position << endl;
		return false;
	}

	if (!are_equal(n_bone_1->get_end_bone_global_pos(current_frame),
			position)) {
		//TODO Better check if position is within range and do not calculate
		//anything if is not
		n_bone_0->quat_arr.at(current_frame) = prev_rot_0;
		n_bone_1->quat_arr.at(current_frame) = prev_rot_1;
		cout << "Can not put joint on coordinates pos doesnt match " << position
				<< endl;
		return false;
	} else {
		return true;
	}
}

float SkeletonFitting::get_swivel_angle(int bone0, int bone1) {
	float swivel_angle = 0.0;
	//Positive direction axis, axis pointing out of the body
	const float Xaxis[] = { 1, 0, 0 };
	//Projection axis, used to determine one of the axis of the local
	//coordinate system
	const float Yaxis[] = { 0, 1, 0 };

	if (!check_bone_index(bone0, bone1)) {
		return swivel_angle;
	}

	osg::Matrix bone_world_matrix_off;
	Node* n_bone_0, *n_bone_1;
	n_bone_0 = skeleton->get_node(bone0);
	n_bone_1 = skeleton->get_node(bone1);
	if (n_bone_0->parent) {
		n_bone_0->parent->get_global_matrix(current_frame,
				bone_world_matrix_off);
	}

	bone_world_matrix_off = osg::Matrix::translate(
			n_bone_0->offset + n_bone_0->froset->at(current_frame))
			* bone_world_matrix_off;
	bone_world_matrix_off = osg::Matrix::inverse(bone_world_matrix_off);

	osg::Vec3 position = n_bone_1->get_end_bone_global_pos(current_frame);

	Matrix T, S;

	osg_to_matrix(T, osg::Matrix::translate(n_bone_0->length));
	osg_to_matrix(S, osg::Matrix::translate(n_bone_1->length));

	SRS s(T, S, Yaxis, Xaxis);

	Matrix G;
	//TODO Final position calculus could be simplified, look through
	//matrices
	osg_to_matrix(G, osg::Matrix::translate(position * bone_world_matrix_off));

	float eangle = 0.0;
	if (s.SetGoal(G, eangle)) {
		position = n_bone_0->get_end_bone_global_pos(current_frame);
		position = position * bone_world_matrix_off;
		swivel_angle = s.PosToAngle(position._v);
	} else {
		cout << "get_swivel_angle() could not recreate bone position" << endl;
	}

	return swivel_angle;
}

float SkeletonFitting::get_median(osg::ref_ptr<osg::Vec3Array> points,
		Skel_Leg use_label, Axis axis) {
	osg::ref_ptr<osg::Vec3Array> aux_vec = new osg::Vec3Array();

	for (unsigned int i = 0; i < points->size(); i++) {
		if (labels[i] == use_label) {
			aux_vec->push_back(points->at(i));
		}
	}

	osg::Vec3Array::iterator first = aux_vec->begin();
	osg::Vec3Array::iterator last = aux_vec->end();
	osg::Vec3Array::iterator middle = first + (last - first) / 2;

	switch (axis) {
	case X:
		std::nth_element(first, middle, last, comp_x);
		return middle->x();
	case Y:
		std::nth_element(first, middle, last, comp_y);
		return middle->y();
	case Z:
		std::nth_element(first, middle, last, comp_z);
		return middle->z();
	}
	return 0.0;
}

float SkeletonFitting::get_mean(osg::ref_ptr<osg::Vec3Array> points,
		Skel_Leg use_label, Axis axis) {

	float mean = 0.0;
	int num_valid = 0;
	for (unsigned int i = 0; i < points->size(); i++) {
		if (labels[i] == use_label) {
			mean += points->at(i)[axis];
			num_valid++;
		}
	}

	if (num_valid > 0) {
		return mean / num_valid;
	} else {
		return 0.0;
	}
}

bool SkeletonFitting::are_equal(const osg::Vec3& v0, const osg::Vec3& v1) {
	if (v0.x() + error_threshold >= v1.x() && v0.x() - error_threshold <= v1.x()
			&& v0.y() + error_threshold >= v1.y()
			&& v0.y() - error_threshold <= v1.y()
			&& v0.z() + error_threshold >= v1.z()
			&& v0.z() - error_threshold <= v1.z()) {
		return true;
	} else {
		return false;
	}
}

void SkeletonFitting::osg_to_matrix(Matrix& dest, const osg::Matrix& orig) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			dest[i][j] = orig(i, j);
		}
	}
}

void SkeletonFitting::matrix_to_osg(osg::Matrix& dest, const Matrix& orig) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			dest(i, j) = orig[i][j];
		}
	}
}

bool SkeletonFitting::check_bone_index(int bone0, int bone1) {
	if (bone0 >= 0 && (unsigned) bone0 < skeleton->get_num_bones() && bone1 >= 0
			&& (unsigned) bone1 < skeleton->get_num_bones()) {
		return true;
	} else {
		return false;
	}
}

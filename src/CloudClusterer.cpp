/*
 * CloudClusterer.cpp
 *
 *  Created on: 12 Feb 2014
 *      Author: m04701
 */

#include "CloudClusterer.h"

CloudClusterer::CloudClusterer() {
	init(0, 0, 0);
}

CloudClusterer::CloudClusterer(int n_frames, int img_rows, int img_cols) {
	init(n_frames, img_rows, img_cols);
}

void CloudClusterer::init(int n_frames, int img_rows, int img_cols) {
	this->n_frames = n_frames;
	this->img_rows = img_rows;
	this->img_cols = img_cols;
	mean_z_front_arr.resize(n_frames, 0);
	mean_z_back_arr.resize(n_frames, 0);
	mean_x_arr.resize(n_frames, 0);
	current_frame = 0;
}

void CloudClusterer::divide_four_sections(
		const osg::ref_ptr<osg::Vec3Array>& point_cloud,
		std::vector<Skeleton::Skel_Leg>& labels, int frame_num,
		bool use_simple_division) {

	cloud = point_cloud;
	current_frame = frame_num;
	labels.clear();
	labels.resize(cloud->size(), Skeleton::Front_Right);

	if (cloud->size() > 4) {
		float mean_y = get_mean(cloud, labels, Skeleton::Front_Right,
				Skeleton::Y);
		int num_invalid = 0;

		//Divide in half vertically, discard all values above
		//The body is larger than the legs so, add a extra
		//so more points are assign to the body
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (cloud->at(i).y() <= mean_y + body_height_extra_threshold) {
				labels[i] = Skeleton::Not_Limbs;
				num_invalid++;
			}
		}

		if (use_simple_division) {
			divide_four_sections_simple(labels);
		} else {
			divide_four_sections_knn(labels, num_invalid);
		}
	}
}

void CloudClusterer::refine_four_sections_division(
		const osg::ref_ptr<osg::Vec3Array>& point_cloud,
		std::vector<Skeleton::Skel_Leg>& labels, int frame_num,
		int head_index) {

	if (cloud->size() > 10) {

		recalculate_front_back_division_side_view(labels, head_index);

		recalculate_right_left_division_mass_center(labels);

		recalculate_right_left_division_front_view(labels, head_index);

		recalculate_right_left_division_back_view(labels, head_index);

		//TODO If knn deletes all the labels of a certain leg then
		//reset the labels vector to its previous state
		recalculate_right_left_knn(labels);

		//Attempt to do some time coherence between frames, code works but
		//the result is the same
		//recalculate_z_division_with_time_coherence();
	}
}

void CloudClusterer::divide_four_sections_simple(
		std::vector<Skeleton::Skel_Leg>& labels) {
	//Divide the remaining values in front/back part along x
	//float mean_x = get_mean(cloud, Skeleton::Front_Right, X);
	float mean_x = get_division_val(cloud, labels, Skeleton::Front_Right,
			Skeleton::X);
	mean_x_arr.at(current_frame) = mean_x;

	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (labels[i] == Skeleton::Front_Right && cloud->at(i).x() <= mean_x) {
			labels[i] = Skeleton::Back_Right;
		}
	}

	//Divide the two groups into left and right
	//Since we are filming the dog from the right side
	//there are less left points than right, so displace the
	//mean point a bit to the left
	//float mean_z_front = get_mean(cloud, Skeleton::Front_Right, Z);
	//float mean_z_back = get_mean(cloud, Skeleton::Back_Right, Z);
	float mean_z_front = get_division_val(cloud, labels, Skeleton::Front_Right,
			Skeleton::Z);
	mean_z_front_arr.at(current_frame) = mean_z_front;
	float mean_z_back = get_division_val(cloud, labels, Skeleton::Back_Right,
			Skeleton::Z);
	mean_z_back_arr.at(current_frame) = mean_z_back;

	for (unsigned int i = 0; i < cloud->size(); i++) {
		if (labels[i] == Skeleton::Front_Right
				&& cloud->at(i).z() >= mean_z_front) {
			labels[i] = Skeleton::Front_Left;
		} else if (labels[i] == Skeleton::Back_Right
				&& cloud->at(i).z() >= mean_z_back) {
			labels[i] = Skeleton::Back_Left;
		}
	}
}

void CloudClusterer::divide_four_sections_knn(
		std::vector<Skeleton::Skel_Leg>& labels, int num_invalid) {
	int num_valid = cloud->size() - num_invalid;
	int max_clusters = 4;
	cv::Mat knn_labels;

	if (num_valid > 4) {

		cv::Mat data2(num_valid, 1, CV_32FC3);
		int j = 0;
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] != Skeleton::Not_Limbs) {
				cv::Point3f ipt(cloud->at(i).x(), cloud->at(i).y(),
						cloud->at(i).z());
				data2.at<cv::Point3f>(j) = ipt;
				j++;
			}
		}

		cv::kmeans(data2, max_clusters, knn_labels,
				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0),
				10, cv::KMEANS_PP_CENTERS);

		j = 0;
		//TODO This division is arbitrary, does not have to correspond
		//with the names, also it would probably be faster to use
		//flann than to use the opencv kmeans method
		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] != Skeleton::Not_Limbs) {
				switch (knn_labels.at<int>(j)) {
				case 0:
					labels[i] = Skeleton::Front_Left;
					break;
				case 1:
					labels[i] = Skeleton::Front_Right;
					break;
				case 2:
					labels[i] = Skeleton::Back_Left;
					break;
				default:
					labels[i] = Skeleton::Back_Right;
					break;
				}
				j++;
			}
		}
	}
}

void CloudClusterer::recalculate_front_back_division_side_view(
		std::vector<Skeleton::Skel_Leg>& labels, int head_index) {
	//Project leg points to front view, using orthogonal projection
	//move from z value
	if (head_index != -1) {

		cv::Mat projected_img(img_rows, img_cols, CV_8U, cv::Scalar(0));
		osg::Vec3 head_pos_trans = -cloud->at(head_index);

		//We want a side view, so no rotation is needed
		//x = [1,0,0]
		//y = [0,1,0]
		//z = [0,0,1]
		//For the projection position the head position is used,
		//but for the legs to be the centre of the projection
		//an offset is needed
		float4x4 invT(0.0);
		invT[0 * 4 + 0] = 1;
		invT[1 * 4 + 1] = 1;
		invT[2 * 4 + 2] = 1;
		invT[3 * 4 + 0] = head_pos_trans.x() + 0.5;
		invT[3 * 4 + 1] = head_pos_trans.y() - 0.05;
		invT[3 * 4 + 2] = head_pos_trans.z() + 1.5;
		invT[3 * 4 + 3] = 1;

		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] != Skeleton::Not_Limbs) {
				float3 point2d = Projections::get_2d_projection(cloud->at(i),
						invT);
				if (point2d.y >= 0 && point2d.y < projected_img.rows
						&& point2d.x >= 0 && point2d.x < projected_img.cols)
					projected_img.at<uchar>(point2d.y, point2d.x) = 255;
			}
		}

		int erosion_size = 2;
		cv::Mat res2 = cv::getStructuringElement(cv::MORPH_RECT,
				cv::Size(4 * erosion_size + 1, erosion_size + 1),
				cv::Point(erosion_size, erosion_size));

		cv::dilate(projected_img, projected_img, res2);

		osg::Vec3 current_div = cloud->at(head_index);
		current_div.x() = mean_x_arr.at(current_frame);
		current_div.y() -= 0.1;

		float depth;
		float3 point2d = Projections::get_2d_projection(current_div, invT,
				depth);

		if (point2d.y >= 0 && point2d.y < projected_img.rows - 1
				&& point2d.x >= 0 && point2d.x < projected_img.cols) {
			int res_row, res_col;
			PixelSearch::cascade_down_left_lower_pixel(projected_img,
					(int) point2d.y, (int) point2d.x, res_row, res_col);

			point2d.y = res_row;
			point2d.x = res_col;

			float4x4 T(0.0);
			T[0 * 4 + 0] = 1;
			T[1 * 4 + 1] = 1;
			T[2 * 4 + 2] = 1;
			T[3 * 4 + 0] = -(head_pos_trans.x() + 0.5);
			T[3 * 4 + 1] = -(head_pos_trans.y() - 0.05);
			T[3 * 4 + 2] = -(head_pos_trans.z() + 1.5);
			T[3 * 4 + 3] = 1;
			//Project back to 3D
			float4 result = Projections::get_3d_projection(point2d, depth, T);
			if (result.x != mean_x_arr.at(current_frame)) {
				mean_x_arr.at(current_frame) = result.x;
				for (unsigned int i = 0; i < cloud->size(); i++) {
					switch (labels[i]) {
					case Skeleton::Front_Right:
						if (cloud->at(i).x() <= result.x) {
							labels[i] = Skeleton::Back_Right;
						}
						break;
					case Skeleton::Front_Left:
						if (cloud->at(i).x() <= result.x) {
							labels[i] = Skeleton::Back_Right;
						} else {
							labels[i] = Skeleton::Front_Right;
						}
						break;
					case Skeleton::Back_Right:
						if (cloud->at(i).x() > result.x) {
							labels[i] = Skeleton::Front_Right;
						}
						break;
					case Skeleton::Back_Left:
						if (cloud->at(i).x() > result.x) {
							labels[i] = Skeleton::Front_Right;
						} else {
							labels[i] = Skeleton::Back_Right;
						}
						break;
					default:
						break;
					}
				}
			}

			//Recalculate left and right divisions with new points
			float mean_z_front = get_division_val(cloud, labels,
					Skeleton::Front_Right, Skeleton::Z);
			mean_z_front_arr.at(current_frame) = mean_z_front;
			float mean_z_back = get_division_val(cloud, labels,
					Skeleton::Back_Right, Skeleton::Z);
			mean_z_back_arr.at(current_frame) = mean_z_back;

			for (unsigned int i = 0; i < cloud->size(); i++) {
				if (labels[i] == Skeleton::Front_Right
						&& cloud->at(i).z() >= mean_z_front) {
					labels[i] = Skeleton::Front_Left;
				} else if (labels[i] == Skeleton::Back_Right
						&& cloud->at(i).z() >= mean_z_back) {
					labels[i] = Skeleton::Back_Left;
				}
			}
		}
	}
}

void CloudClusterer::recalculate_right_left_division_time_coherence(
		std::vector<Skeleton::Skel_Leg>& labels) {
	float mean_z_front = mean_z_front_arr.at(current_frame);
	float mean_z_back = mean_z_back_arr.at(current_frame);
	int front_valid = 1, back_valid = 1;

	if (current_frame > 0 && mean_z_front_arr.at(current_frame - 1) != 0.0) {
		mean_z_front += mean_z_front_arr.at(current_frame - 1);
		front_valid++;
	}
	if (current_frame > 0 && mean_z_back_arr.at(current_frame - 1) != 0.0) {
		mean_z_back += mean_z_back_arr.at(current_frame - 1);
		back_valid++;
	}
	if (current_frame < n_frames - 1
			&& mean_z_front_arr.at(current_frame + 1) != 0.0) {
		mean_z_front += mean_z_front_arr.at(current_frame + 1);
		front_valid++;
	}
	if (current_frame < n_frames - 1
			&& mean_z_back_arr.at(current_frame + 1) != 0.0) {
		mean_z_back += mean_z_back_arr.at(current_frame + 1);
		back_valid++;
	}

	//Calculate mean of previous and next frame z value
	mean_z_front = mean_z_front / front_valid;
	mean_z_back = mean_z_back / back_valid;

	//New z division is combination of current and previous/next frame
	mean_z_front = mean_z_front * 0.7
			+ mean_z_front_arr.at(current_frame) * 0.3;
	mean_z_back = mean_z_back * 0.7 + mean_z_back_arr.at(current_frame) * 0.3;
	//Redo division using new z value
	reclassify_left_right_leg_points(labels, mean_z_front, mean_z_back);
}

void CloudClusterer::recalculate_right_left_division_mass_center(
		std::vector<Skeleton::Skel_Leg>& labels) {
	//Calculate the mean z of each leg
	//Then recalculate what points belong to each leg, if the
	//calculation changes any point then do it again
	//This fixes errors in two frames
	bool point_relabeled = true;
	while (point_relabeled) {
		point_relabeled = false;
		float mean_z_front = (get_mean(cloud, labels, Skeleton::Front_Right,
				Skeleton::Z)
				+ get_mean(cloud, labels, Skeleton::Front_Left, Skeleton::Z))
				* 0.5;
		mean_z_front_arr.at(current_frame) = mean_z_front;

		float mean_z_back = (get_mean(cloud, labels, Skeleton::Back_Right,
				Skeleton::Z)
				+ get_mean(cloud, labels, Skeleton::Back_Left, Skeleton::Z))
				* 0.5;
		mean_z_back_arr.at(current_frame) = mean_z_back;

		point_relabeled = reclassify_left_right_leg_points(labels, mean_z_front,
				mean_z_back);
	}
}

void CloudClusterer::recalculate_right_left_division_front_view(
		std::vector<Skeleton::Skel_Leg>& labels, int head_index) {
	//Project leg points to front view, using orthogonal projection
	//move from z value
	if (head_index != -1) {

		cv::Mat projected_img(img_rows, img_cols, CV_8U, cv::Scalar(0));
		osg::Vec3 head_pos_trans = -cloud->at(head_index);

		//We want a front view so in world axes is vectors
		//x = [0,0,1]
		//y = [0,1,0]
		//z = [-1,0,0]
		//For the projection position the head position is used,
		//but for the legs to be the centre of the projection
		//an offset is needed
		float4x4 invT(0.0);
		invT[0 * 4 + 2] = -1;
		invT[1 * 4 + 1] = 1;
		invT[2 * 4 + 0] = 1;
		invT[3 * 4 + 0] = head_pos_trans.z() + 0.05;
		invT[3 * 4 + 1] = head_pos_trans.y() - 0.15;
		invT[3 * 4 + 2] = -(head_pos_trans.x() - 0.3);
		invT[3 * 4 + 3] = 1;

		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] == Skeleton::Front_Left
					|| labels[i] == Skeleton::Front_Right) {
				float3 point2d = Projections::get_2d_projection(cloud->at(i),
						invT);
				if (point2d.y >= 0 && point2d.y < projected_img.rows
						&& point2d.x >= 0 && point2d.x < projected_img.cols)
					projected_img.at<uchar>(point2d.y, point2d.x) = 255;
			}
		}

		int erosion_size = 2;
		cv::Mat res2 = cv::getStructuringElement(cv::MORPH_RECT,
				cv::Size(4 * erosion_size + 1, erosion_size + 1),
				cv::Point(erosion_size, erosion_size));

		cv::dilate(projected_img, projected_img, res2);

		osg::Vec3 current_div = cloud->at(head_index);
		current_div.y() += 0.09;
		current_div.z() = mean_z_front_arr.at(current_frame);
		float depth;
		float3 point2d = Projections::get_2d_projection(current_div, invT,
				depth);

		if (point2d.y >= 0 && point2d.y < projected_img.rows - 1
				&& point2d.x >= 0 && point2d.x < projected_img.cols) {
			int res_row, res_col;
			PixelSearch::cascade_down_right_lower_pixel(projected_img,
					(int) point2d.y, (int) point2d.x, res_row, res_col);
			point2d.y = res_row;
			point2d.x = res_col;

			float4x4 T(0.0);
			T[0 * 4 + 2] = 1;
			T[1 * 4 + 1] = 1;
			T[2 * 4 + 0] = -1;
			T[3 * 4 + 0] = -(head_pos_trans.x() - 0.3);
			T[3 * 4 + 1] = -(head_pos_trans.y() - 0.15);
			T[3 * 4 + 2] = -(head_pos_trans.z() + 0.05);
			T[3 * 4 + 3] = 1;
			//Project back to 3D
			float4 result = Projections::get_3d_projection(point2d, depth, T);
			if (result.z != mean_z_front_arr.at(current_frame)) {
				mean_z_front_arr.at(current_frame) = result.z;
				for (unsigned int i = 0; i < cloud->size(); i++) {
					switch (labels[i]) {
					case Skeleton::Front_Right:
						if (cloud->at(i).z() > result.z) {
							labels[i] = Skeleton::Front_Left;
						}
						break;
					case Skeleton::Front_Left:
						if (cloud->at(i).z() <= result.z) {
							labels[i] = Skeleton::Front_Right;
						}
						break;
					default:
						break;
					}
				}
			}
		}
	}
}

void CloudClusterer::recalculate_right_left_division_back_view(
		std::vector<Skeleton::Skel_Leg>& labels, int head_index) {
	//Project leg points to front view, using orthogonal projection
	//move from z value
	if (head_index != -1) {

		cv::Mat projected_img(img_rows, img_cols, CV_8U, cv::Scalar(0));
		osg::Vec3 head_pos_trans = -cloud->at(head_index);

		//We want a front view so in world axes is vectors
		//x = [0,0,1]
		//y = [0,1,0]
		//z = [-1,0,0]
		//For the projection position the head position is used,
		//but for the legs to be the centre of the projection
		//an offset is needed
		float4x4 invT(0.0);
		invT[0 * 4 + 2] = -1;
		invT[1 * 4 + 1] = 1;
		invT[2 * 4 + 0] = 1;
		invT[3 * 4 + 0] = head_pos_trans.z() + 0.05;
		invT[3 * 4 + 1] = head_pos_trans.y() - 0.15;
		invT[3 * 4 + 2] = -(head_pos_trans.x() - 0.3);
		invT[3 * 4 + 3] = 1;

		for (unsigned int i = 0; i < cloud->size(); i++) {
			if (labels[i] == Skeleton::Back_Left
					|| labels[i] == Skeleton::Back_Right) {
				float3 point2d = Projections::get_2d_projection(cloud->at(i),
						invT);
				if (point2d.y >= 0 && point2d.y < projected_img.rows
						&& point2d.x >= 0 && point2d.x < projected_img.cols)
					projected_img.at<uchar>(point2d.y, point2d.x) = 255;
			}
		}

		int erosion_size = 2;
		cv::Mat res2 = cv::getStructuringElement(cv::MORPH_RECT,
				cv::Size(4 * erosion_size + 1, erosion_size + 1),
				cv::Point(erosion_size, erosion_size));

		cv::dilate(projected_img, projected_img, res2);

		osg::Vec3 current_div = cloud->at(head_index);
		current_div.y() += 0.09;
		current_div.z() = mean_z_back_arr.at(current_frame);
		float depth;
		float3 point2d = Projections::get_2d_projection(current_div, invT,
				depth);

		if (point2d.y >= 0 && point2d.y < projected_img.rows - 1
				&& point2d.x >= 0 && point2d.x < projected_img.cols) {
			int res_row, res_col;
			PixelSearch::cascade_down_right_lower_pixel(projected_img,
					(int) point2d.y, (int) point2d.x, res_row, res_col);

			point2d.y = res_row;
			point2d.x = res_col;

			float4x4 T(0.0);
			T[0 * 4 + 2] = 1;
			T[1 * 4 + 1] = 1;
			T[2 * 4 + 0] = -1;
			T[3 * 4 + 0] = -(head_pos_trans.x() - 0.3);
			T[3 * 4 + 1] = -(head_pos_trans.y() - 0.15);
			T[3 * 4 + 2] = -(head_pos_trans.z() + 0.05);
			T[3 * 4 + 3] = 1;
			//Project back to 3D
			float4 result = Projections::get_3d_projection(point2d, depth, T);
			if (result.z != mean_z_back_arr.at(current_frame)) {
				mean_z_back_arr.at(current_frame) = result.z;
				for (unsigned int i = 0; i < cloud->size(); i++) {
					switch (labels[i]) {
					case Skeleton::Back_Right:
						if (cloud->at(i).z() > result.z) {
							labels[i] = Skeleton::Back_Left;
						}
						break;
					case Skeleton::Back_Left:
						if (cloud->at(i).z() <= result.z) {
							labels[i] = Skeleton::Back_Right;
						}
						break;
					default:
						break;
					}
				}
			}
		}
	}
}

bool CloudClusterer::reclassify_left_right_leg_points(
		std::vector<Skeleton::Skel_Leg>& labels, float mean_z_front,
		float mean_z_back) {
	bool point_relabeled = false;
	for (unsigned int i = 0; i < cloud->size(); i++) {
		switch (labels[i]) {
		case Skeleton::Front_Right:
			if (cloud->at(i).z() > mean_z_front) {
				labels[i] = Skeleton::Front_Left;
				point_relabeled = true;
			}
			break;
		case Skeleton::Front_Left:
			if (cloud->at(i).z() <= mean_z_front) {
				labels[i] = Skeleton::Front_Right;
				point_relabeled = true;
			}
			break;
		case Skeleton::Back_Right:
			if (cloud->at(i).z() > mean_z_back) {
				labels[i] = Skeleton::Back_Left;
				point_relabeled = true;
			}
			break;
		case Skeleton::Back_Left:
			if (cloud->at(i).z() <= mean_z_back) {
				labels[i] = Skeleton::Back_Right;
				point_relabeled = true;
			}
			break;
		default:
			break;
		}
	}
	return point_relabeled;
}

void CloudClusterer::recalculate_right_left_knn(
		std::vector<Skeleton::Skel_Leg>& labels, unsigned int num_nn,
		float max_distance_threshold, unsigned int max_ite) {

	std::vector<std::vector<int> > indices;
	std::vector<std::vector<float> > dists;
	unsigned int num_ex_it = 0;
	bool bad_division = true;

	//Loop while the current division is not right
	while (bad_division && num_ex_it < max_ite) {

		if (cloud->size() < num_nn) {
			return;
		}

		indices.clear();
		dists.clear();

		//Calculate the nearest neighbours of all the leg points
		if (!knn_searcher.knn_search(cloud, num_nn, indices, dists)) {
			return;
		}

		std::vector<Skeleton::Skel_Leg> labels_new(labels);
		unsigned int num_ite = 0;
		bool points_moved;
		do {
			points_moved = false;
			for (unsigned int i = 0; i < cloud->size(); i++) {
				int same_leg_neighbours[4] = { 0, 0, 0, 0 };
				int total_valid_neighbours[4] = { 0, 0, 0, 0 };
				for (unsigned int j = 0; j < num_nn; j++) {
					switch (labels[i]) {
					case Skeleton::Front_Right:
						if (labels[indices[i][j]] == Skeleton::Front_Right
								&& dists[i][j] < max_distance_threshold) {
							same_leg_neighbours[0]++;
							total_valid_neighbours[0]++;
						} else if (labels[indices[i][j]] == Skeleton::Front_Left
								&& dists[i][j] < max_distance_threshold) {
							total_valid_neighbours[0]++;
						}
						break;
					case Skeleton::Front_Left:
						if (labels[indices[i][j]] == Skeleton::Front_Left
								&& dists[i][j] < max_distance_threshold) {
							same_leg_neighbours[1]++;
							total_valid_neighbours[1]++;
						} else if (labels[indices[i][j]]
								== Skeleton::Front_Right
								&& dists[i][j] < max_distance_threshold) {
							total_valid_neighbours[1]++;
						}
						break;
					case Skeleton::Back_Right:
						if (labels[indices[i][j]] == Skeleton::Back_Right
								&& dists[i][j] < max_distance_threshold) {
							same_leg_neighbours[2]++;
							total_valid_neighbours[2]++;
						} else if (labels[indices[i][j]] == Skeleton::Back_Left
								&& dists[i][j] < max_distance_threshold) {
							total_valid_neighbours[2]++;
						}
						break;
					case Skeleton::Back_Left:
						if (labels[indices[i][j]] == Skeleton::Back_Left
								&& dists[i][j] < max_distance_threshold) {
							same_leg_neighbours[3]++;
							total_valid_neighbours[3]++;
						} else if (labels[indices[i][j]] == Skeleton::Back_Right
								&& dists[i][j] < max_distance_threshold) {
							total_valid_neighbours[3]++;
						}
						break;
					default:
						break;
					}
				}

				switch (labels[i]) {
				case Skeleton::Front_Right:
					if (total_valid_neighbours[0] > 0
							&& same_leg_neighbours[0]
									< total_valid_neighbours[0] / 2.0) {
						labels_new[i] = Skeleton::Front_Left;
						points_moved = true;
					}
					break;
				case Skeleton::Front_Left:
					if (total_valid_neighbours[1] > 0
							&& same_leg_neighbours[1]
									< total_valid_neighbours[1] / 2.0) {
						labels_new[i] = Skeleton::Front_Right;
						points_moved = true;
					}
					break;
				case Skeleton::Back_Right:
					if (total_valid_neighbours[2] > 0
							&& same_leg_neighbours[2]
									< total_valid_neighbours[2] / 2.0) {
						labels_new[i] = Skeleton::Back_Left;
						points_moved = true;
					}
					break;
				case Skeleton::Back_Left:
					if (total_valid_neighbours[3] > 0
							&& same_leg_neighbours[3]
									< total_valid_neighbours[3] / 2.0) {
						labels_new[i] = Skeleton::Back_Right;
						points_moved = true;
					}
					break;
				default:
					break;
				}

			}
			labels = labels_new;
			num_ite++;
		} while (points_moved && num_ite < max_ite);

		bad_division = !knn_division_done(labels, num_nn, indices, dists);
		num_nn += 10;
		max_distance_threshold += 0.005;
		num_ex_it++;
	}
}

bool CloudClusterer::knn_division_done(std::vector<Skeleton::Skel_Leg>& labels,
		unsigned int num_nn, const std::vector<std::vector<int> >& indices,
		const std::vector<std::vector<float> >& dists,
		float min_distance_treshold) {

	//If any point of the other leg is closer than a threshold then
	//the division is not finished yet
	for (unsigned int i = 0; i < cloud->size(); i++) {
		for (unsigned int j = 0; j < num_nn; j++) {
			switch (labels[i]) {
			case Skeleton::Front_Right:
				if (labels[indices[i][j]] == Skeleton::Front_Left
						&& dists[i][j] < min_distance_treshold) {
					return false;
				}
				break;
			case Skeleton::Front_Left:
				if (labels[indices[i][j]] == Skeleton::Front_Right
						&& dists[i][j] < min_distance_treshold) {
					return false;
				}
				break;
			case Skeleton::Back_Right:
				if (labels[indices[i][j]] == Skeleton::Back_Left
						&& dists[i][j] < min_distance_treshold) {
					return false;
				}
				break;
			case Skeleton::Back_Left:
				if (labels[indices[i][j]] == Skeleton::Back_Right
						&& dists[i][j] < min_distance_treshold) {
					return false;
				}
				break;
			default:
				break;
			}
		}
	}

	return true;
}

float CloudClusterer::get_mean(osg::ref_ptr<osg::Vec3Array> points,
		std::vector<Skeleton::Skel_Leg>& labels, Skeleton::Skel_Leg use_label,
		Skeleton::Axis axes) {

	float mean = 0.0;
	int num_valid = 0;
	for (unsigned int i = 0; i < points->size(); i++) {
		if (labels[i] == use_label) {
			mean += points->at(i)[axes];
			num_valid++;
		}
	}

	if (num_valid > 0) {
		return mean / num_valid;
	} else {
		return 0.0;
	}
}

//This method returns the mean value between the two most distant
//points, it does not solve the problems of the simple division either
float CloudClusterer::get_division_val(osg::ref_ptr<osg::Vec3Array> points,
		std::vector<Skeleton::Skel_Leg>& labels, Skeleton::Skel_Leg use_label,
		Skeleton::Axis axes) {

	osg::ref_ptr<osg::Vec3Array> points_copy = new osg::Vec3Array;
	for (unsigned int i = 0; i < points->size(); i++) {
		if (labels[i] == use_label) {
			points_copy->push_back(points->at(i));
		}
	}

	if (points_copy->size() >= 2) {
		bool (*comp_funct)(const osg::Vec3&, const osg::Vec3&);
		switch (axes) {
		case Skeleton::X:
			comp_funct = CloudClusterer::comp_x;
			break;
		case Skeleton::Y:
			comp_funct = CloudClusterer::comp_y;
			break;
		default:
			comp_funct = CloudClusterer::comp_z;
			break;
		}
		sortstruct s(this, comp_funct);
		std::sort(points_copy->begin(), points_copy->end(), s);
		return (points_copy->front()[axes] + points_copy->back()[axes]) * 0.5;
	} else {
		return 0.0;
	}
}

bool CloudClusterer::comp_x(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.x() < j.x());
}

bool CloudClusterer::comp_y(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.y() < j.y());
}

bool CloudClusterer::comp_z(const osg::Vec3& i, const osg::Vec3& j) {
	return (i.z() < j.z());
}

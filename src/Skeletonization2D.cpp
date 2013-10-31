#include "Skeletonization2D.h"

using std::cout;
using std::endl;

Skeletonization2D::Skeletonization2D(
		const boost::shared_ptr<RGBD_Camera> camera_, int max_clusters_) :
		camera(camera_), max_clusters(max_clusters_) {
	generate_skeletonization();
}

Skeletonization2D::~Skeletonization2D() {

}

const cv::Mat* const Skeletonization2D::get_frame(const int frame_num) const {
	return &(skeletonized_frames[frame_num]);
}

void Skeletonization2D::generate_skeletonization() {
	int begin = camera->get_first_frame_num();
	int end = camera->get_last_frame_num();
	skeletonized_frames.reserve(end - begin);
	for (int i = begin; i <= end; i++) {
		skeletonized_frames.push_back(
				dist_transform_skeletonization(camera->get_depth_map(i)));
	}
}

//Generates a vector of 2D points from a binary image
osg::ref_ptr<osg::Vec3Array> Skeletonization2D::points_from_image(
		const cv::Mat& seg_img) {
	int rows = seg_img.rows;
	int cols = seg_img.cols;
	//Maximun size is this, but it would be good to reserve just the neede space
	osg::ref_ptr<osg::Vec3Array> skeleton_points = new osg::Vec3Array();
	//We are going to put some elements in the vector, better reserve some
	//space to make it faster
	skeleton_points->reserve(100);

	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			//If point is not background, then save it
			if ((int) seg_img.at<uchar>(row, col) == 255) {
				skeleton_points->push_back(osg::Vec3(row, col, 0));
			}
		}
	}
	return skeleton_points.get();
}

//Generates a skeletonized image from a depth image
cv::Mat Skeletonization2D::dist_transform_skeletonization(
		const cv::Mat* seg_img) {

	int rows = seg_img->rows;
	int cols = seg_img->cols;

	cv::Mat res, temp1, temp2;
	//Make binary image of segmented depth map:
	//---------------------
	cv::Mat bin_img(rows, cols, CV_8U);
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			//If any of the 4 neighbours are inconsistent with the central pixel,
			//set it to background to force an edge there:
			int threshold = 40;			//40mm
			bool has_consistent_4_neighbours = true;
			if (row > 0
					&& abs(
							(int) seg_img->at<ushort>(row, col)
									- (int) seg_img->at<ushort>(row - 1, col))
							> threshold) {
				has_consistent_4_neighbours = false;
			}
			if (row < rows - 1
					&& abs(
							(int) seg_img->at<ushort>(row, col)
									- (int) seg_img->at<ushort>(row + 1, col))
							> threshold) {
				has_consistent_4_neighbours = false;
			}
			if (col > 0
					&& abs(
							(int) seg_img->at<ushort>(row, col)
									- (int) seg_img->at<ushort>(row, col - 1))
							> threshold) {
				has_consistent_4_neighbours = false;
			}
			if (col < cols - 1
					&& abs(
							(int) seg_img->at<ushort>(row, col)
									- (int) seg_img->at<ushort>(row, col + 1))
							> threshold) {
				has_consistent_4_neighbours = false;
			}

			if (seg_img->at<ushort>(row, col) != 0
					&& has_consistent_4_neighbours) {
				bin_img.at<uchar>(row, col) = 255;
			} else {
				bin_img.at<uchar>(row, col) = 0;
			}
		}
	}

	//Perform distance transform:
	//---------------------

	cv::distanceTransform(bin_img, temp2, CV_DIST_L2, CV_DIST_MASK_PRECISE);

	//2nd derivative magnitude image:
	//---------------------

	cv::Sobel(temp2, temp1, CV_32F, 2, 0, 1);
	cv::Sobel(temp2, res, CV_32F, 0, 2, 1);

	cv::multiply(temp1, temp1, temp2);
	cv::multiply(res, res, temp1);

	cv::sqrt(temp2 + temp1, res);	//abs(diff_xx) + abs(diff_yy);

	//Threshold
	//---------------------
	cv::threshold(res, temp1, 0.7, 255, CV_8U);
	cv::Mat thresh_8bit;
	temp1.convertTo(thresh_8bit, CV_8U);
	//---------------------

	//Erode binary image
	//---------------------
	int erosion_size = 2;
	res = cv::getStructuringElement(cv::MORPH_ELLIPSE,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			cv::Point(erosion_size, erosion_size));

	cv::erode(bin_img, temp1, res);
	cv::bitwise_and(thresh_8bit, temp1, res);
	temp1 = connectivity_preserving_thinning(res);
	res = remove_isolated_short_segments(temp1, 5);
	temp1 = connectivity_preserving_thinning(res);
	res = remove_isolated_short_segments(temp1, 15);
	delete_arm(res);

	return res;
}

cv::Mat Skeletonization2D::remove_isolated_short_segments(cv::Mat& img_in,
		unsigned int thresh_length) {
	cv::Mat result = img_in.clone();

	cv::Mat free_nodes(result.rows, result.cols, CV_8U, cv::Scalar(0));
	cv::Mat junction_nodes(result.rows, result.cols, CV_8U, cv::Scalar(0));

	//Make map of end points and junctions:
	for (int row = 1; row < result.rows - 1; row++) {
		for (int col = 1; col < result.cols - 1; col++) {
			//Check that pixel is occupied:
			if (result.at<uchar>(row, col) != 0) {
				//Get number of neighbours:
				int num_neigbours = 0;
				for (int d_row = -1; d_row <= 1; d_row++) {
					for (int d_col = -1; d_col <= 1; d_col++) {
						if (d_row == 0 && d_col == 0) {
							continue;	//(don't count central pixel)
						}
						if (result.at<uchar>(row + d_row, col + d_col) != 0) {
							num_neigbours++;
						}
					}
				}
				//Immediately remove isolated pixels:
				if (num_neigbours == 0) {
					result.at<uchar>(row, col) = 0;
				}
				//Free node:
				else if (num_neigbours == 1) {
					free_nodes.at<uchar>(row, col) = 255;
				}
				//Junction node:
				else if (num_neigbours > 2) {
					junction_nodes.at<uchar>(row, col) = 255;
				}

			}
		}
	}

	//DEBUG: Show the nodes images:
	//cv::imshow("Free nodes", free_nodes);
	//cv::imshow("Junction nodes", junction_nodes);

	//For all the free nodes,
	for (int row = 1; row < result.rows - 1; row++) {
		for (int col = 1; col < result.cols - 1; col++) {
			if (free_nodes.at<uchar>(row, col) != 0) {
				//Walk along until a another free node or a junction node is found
				std::vector<cv::Point> path;
				int walk_row = row;
				int walk_col = col;
				path.push_back(cv::Point(walk_col, walk_row));
				bool next_node_found = false;
				//while(next_node_found == false)
				for (int iter = 0; iter < 100; iter++) {
					if (next_node_found == true) {
						break;
					}

					bool next_point_found = false;
					for (int d_row = -1; d_row <= 1; d_row++) {
						for (int d_col = -1; d_col <= 1; d_col++) {
							if ((d_row == 0 && d_col == 0)
									|| //(don't count central pixel)
									//(path[path.size()-1].x == walk_col + d_col
									//&& path[path.size()-1].y == walk_row + d_row) )
									path[path.size() - 1]
											== cv::Point(walk_col + d_col,
													walk_row + d_row)
									||		//(don't go where you've just been)
									(path.size() > 1
											&& path[path.size() - 2]
													== cv::Point(
															walk_col + d_col,
															walk_row + d_row))
									|| next_point_found == true)//(don't keep looking if the next point has already been found)
											{
								continue;
							}
							if (result.at<uchar>(walk_row + d_row,
									walk_col + d_col) != 0) {
								walk_row += d_row;
								walk_col += d_col;

								path.push_back(cv::Point(walk_col, walk_row));
								next_point_found = true;

								//Check if another node has been found:
								if ((free_nodes.at<uchar>(walk_row, walk_col)
										!= 0
										|| junction_nodes.at<uchar>(walk_row,
												walk_col) != 0)) {
									next_node_found = true;
									//DEBUG:
									//if(free_nodes.at<uchar>(walk_row, walk_col) != 0)
									//cout << "Found free node" << endl;
									//if(junction_nodes.at<uchar>(walk_row, walk_col) != 0)
									//cout << "Found junction node" << endl;

									//Delete last element so that breaks are not introduced at junctions
									path.pop_back();
								}

							}
						}
					}
				}
				//DEBUG:

				//cout << "(next node " << std::string(next_node_found?"found":"not found") << ") path length = " << path.size() << endl;
				//for(int p = 0; p<path.size(); p++)
				//{
				//cout << path[p] << endl;
				//}

				//Check length of path, deleting points if it's shorted than threshold:
				if (path.size() < thresh_length) {
					for (unsigned int i = 0; i < path.size(); i++) {
						result.at<uchar>(path[i].y, path[i].x) = 0;
					}
				}

			}
		}
	}

	//If the distance walked is less than the threshold, delete the node and all points between it and the node found

	return result;
}

cv::Mat Skeletonization2D::connectivity_preserving_thinning(cv::Mat& img_in) {
	cv::Mat result = img_in.clone();

	//Two iterations should be sufficient (TODO - rather check if result changed between iterations
	bool was_updated_this_iter = true;
	int iter = 0;
	while (was_updated_this_iter == true) {
		was_updated_this_iter = false;
		iter++;

		for (int row = 1; row < result.rows - 1; row++) {
			for (int col = 1; col < result.cols - 1; col++) {
				//Check that pixel is occupied:
				if (result.at<uchar>(row, col) != 0) {
					//Establish original connectivity:
					//(it is trivial that all 'on' pixels are known to be connected each other
					//when the central pixel is 'on')

					//Establish connectivity if removed:
					//Check that number of connected components is still 1:
					int seed_d_row = -2;
					int seed_d_col = -2;
					bool found_seed = false;
					for (int d_row = -1; d_row <= 1; d_row++) {
						if (found_seed) {
							break;
						}
						for (int d_col = -1; d_col <= 1; d_col++) {
							if (d_row == 0 && d_col == 0) {
								continue;
							}
							if (result.at<uchar>(row + d_row, col + d_col)
									!= 0) {
								seed_d_row = d_row;
								seed_d_col = d_col;
								found_seed = true;
								break;
							}
						}
					}
					//(Note: seed is the top-left-most 'on' pixel)

					//Set central pixel to 'off':
					result.at<uchar>(row, col) = 0;

					int num_in_win = 0;
					int num_con = 0;

					if (found_seed) {
						cv::Mat propergated(3, 3, CV_8U, cv::Scalar(0));

						for (int s_row = -1; s_row <= 1; s_row++) {
							for (int s_col = -1; s_col <= 1; s_col++) {
								if (result.at<uchar>(row + s_row, col + s_col)
										!= 0) {
									num_in_win++;
								}
							}
						}

						//Set seed pixel to 'on': (adding (1,1) offset to get beteen (-1,-1) and (0,0) origin)
						propergated.at<uchar>(seed_d_row + 1, seed_d_col + 1) =
								255;

						//Iterate thrice to cover the 3x3 region:
						for (int iter = 0; iter < 3; iter++) {
							//Propagate using 8-connectivity from seed:
							for (int s_row = 0; s_row < 3; s_row++) {
								for (int s_col = 0; s_col < 3; s_col++) {
									if (propergated.at<uchar>(s_row, s_col)
											!= 0) {
										for (int t_d_row = -1; t_d_row <= 1;
												t_d_row++) {
											for (int t_d_col = -1; t_d_col <= 1;
													t_d_col++) {
												if ((s_row + t_d_row >= 0
														&& s_row + t_d_row < 3
														&& s_col + t_d_col >= 0
														&& s_col + t_d_col < 3)	//check bounds
														&& result.at<uchar>(
																row + s_row
																		+ t_d_row
																		- 1,
																col + s_col
																		+ t_d_col
																		- 1)
																!= 0) {
													propergated.at<uchar>(
															s_row + t_d_row,
															s_col + t_d_col) =
															255;
												}
											}
										}
									}

								}
							}
						}
						for (int s_row = 0; s_row < 3; s_row++) {
							for (int s_col = 0; s_col < 3; s_col++) {
								if (propergated.at<uchar>(s_row, s_col) != 0) {
									num_con++;
								}
							}
						}
					}

					//Set on again if neighbours get disconnected, or if it has fewer than 2 neighours
					//(in which case it is the end of a line, which must not be shortened):
					if (num_con != num_in_win || num_con < 2) {
						//Set central pixel back to 'on':
						result.at<uchar>(row, col) = 255;

						//DEBUG:
						//cout << "num_con: " << num_con << ", num_in_win: " << num_in_win << endl;
					} else {
						was_updated_this_iter = true;
					}

				}
			}
		}

	}

	//DEBUG:
	//cout << "Skeleton thinning done in " << iter << " iterations" << endl;

	return result;
}

void Skeletonization2D::delete_arm(cv::Mat& img_in) {
	cv::Mat labels;
	cv::Mat centers(max_clusters, 1, CV_32FC2);
	cv::Mat data(img_in.cols * img_in.rows, 1, CV_32FC2);

	int total_points = 0;
	for (int row = 0; row < img_in.rows; row++) {
		for (int col = 0; col < img_in.cols; col++) {
			if (img_in.at<uchar>(row, col) == 255) {
				cv::Point ipt(row, col);
				data.at<cv::Point2f>(total_points) = ipt;
				total_points++;
			}
		}
	}

	cv::Mat data2(total_points, 1, CV_32FC2);
	for (int i = 0; i < total_points; i++) {
		data2.at<cv::Point2f>(i) = data.at<cv::Point2f>(i);
	}

	//Kmeans wants:
	// - data -> a vector of points
	// - K -> a number of clusters
	// - bestLabels -> it will return in labels an array that indicates with
	// an integer to which cluster goes each point in data
	// - criteria -> a termination criteria, in this case we indicate an epsilon
	// 1.0 ( how good the solution is) and a number of maximum iterations 10
	// - attempts -> how many times the algorithm is executed using different
	// initial labellings
	// - flags -> how the centers are selected on first iteration
	// - centers -> output array with the center point of each cluster
	// for more info -> http://docs.opencv.org/modules/core/doc/clustering.html?highlight=kmeans
	cv::kmeans(data2, max_clusters, labels,
			cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3,
			cv::KMEANS_PP_CENTERS, centers);

	//Find the closer to top cluster
	int distance = img_in.rows, arm_cluster = 0;

	for (int i = 0; i < centers.rows; i++) {
		if (centers.at<cv::Point2f>(i).x < distance) {
			distance = centers.at<cv::Point2f>(i).x;
			arm_cluster = i;
		}
	}

	//Delete all the pixels that belong to that cluster
	cv::Mat img_out = img_in.clone();
	distance = 0;
	int lowest_index = 0;

	for (int i = 0; i < labels.rows; i++) {
		if (labels.at<int>(i) == arm_cluster) {
			int row = data2.at<cv::Point2f>(i).x;
			int col = data2.at<cv::Point2f>(i).y;
			img_in.at<uchar>(row, col) = 0;
			if (row > distance) {
				distance = row;
				lowest_index = i;
			}
		}
	}

	//Check if the lowest pixel in the cluster that was deleted belong to a line
	//if it did then continue deleting until the line ends.
	int row, col, next_row, next_col;
	row = data2.at<cv::Point2f>(lowest_index).x;
	col = data2.at<cv::Point2f>(lowest_index).y;

	while (get_neighbor_white_pixel(img_in, row, col, next_row, next_col)) {
		img_in.at<uchar>(next_row, next_col) = 0;
		row = next_row;
		col = next_col;
	}
}

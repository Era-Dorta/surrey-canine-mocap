#include "Skeletonization.h"

using std::cout;
using std::endl;

Skeletonization::Skeletonization(std::map<int, RGBD_Frame>* camera_frames)
{
	frames = camera_frames;
}

Skeletonization::~Skeletonization()
{
	frames = NULL;
}

void Skeletonization::generate_skeletonization()
{
	std::map<int, RGBD_Frame>::iterator i(frames->begin());
	for(; i != frames->end(); ++i ){
		skeletonized_imgs.push_back(dist_transform_skeletonization(i->second.depth_img));
	}
}

cv::Mat Skeletonization::dist_transform_skeletonization(cv::Mat& seg_img)
{

	int rows = seg_img.rows;
	int cols = seg_img.cols;

	//Make binary image of segmented depth map:
	//---------------------
	cv::Mat bin_img(rows, cols, CV_8U);
	for(int row = 0; row < rows; row++)
	{
		for(int col = 0; col < cols; col++)
		{
			//If any of the 4 neighbours are inconsistent with the central pixel,
			//set it to background to force an edge there:
			int threshold = 40;//40mm
			bool has_consistent_4_neighbours = true;
			if(row>0 && abs((int)seg_img.at<ushort>(row, col) - (int)seg_img.at<ushort>(row-1, col)) > threshold)
			{
				has_consistent_4_neighbours = false;
			}
			if(row<rows-1 && abs((int)seg_img.at<ushort>(row, col) - (int)seg_img.at<ushort>(row+1, col)) > threshold)
			{
				has_consistent_4_neighbours = false;
			}
			if(col>0 && abs((int)seg_img.at<ushort>(row, col) - (int)seg_img.at<ushort>(row, col-1)) > threshold)
			{
				has_consistent_4_neighbours = false;
			}
			if(col<cols-1 && abs((int)seg_img.at<ushort>(row, col) - (int)seg_img.at<ushort>(row, col+1)) > threshold)
			{
				has_consistent_4_neighbours = false;
			}

			if(seg_img.at<ushort>(row, col) != 0 &&
					has_consistent_4_neighbours)
			{
				bin_img.at<uchar>(row, col) = 255;
			}
			else
			{
				bin_img.at<uchar>(row, col) = 0;
			}
		}
	}

	//cv::imshow("Binary image", bin_img);

	//---------------------

	//Perform distance transform:
	//---------------------
	cv::Mat dist_transform_img;

	cv::distanceTransform(bin_img, dist_transform_img, CV_DIST_L2, CV_DIST_MASK_PRECISE);

	//cv::imshow("Dist transform", dist_transform_img*0.01f);

	//---------------------

	//2nd derivative magnitude image:
	//---------------------
	cv::Mat diff_xx;
	cv::Mat diff_yy;

	cv::Sobel(dist_transform_img, diff_xx, CV_32F, 2, 0, 1);
	cv::Sobel(dist_transform_img, diff_yy, CV_32F, 0, 2, 1);

	cv::Mat grad_dist_xform;
	cv::Mat diff_xx_sq;
	cv::multiply(diff_xx, diff_xx, diff_xx_sq);
	cv::Mat diff_yy_sq;
	cv::multiply(diff_yy, diff_yy, diff_yy_sq);

	cv::sqrt(diff_xx_sq + diff_yy_sq, grad_dist_xform);//abs(diff_xx) + abs(diff_yy);

	//cv::imshow("2nd deriv dist transform", grad_dist_xform/10);

	//---------------------

	//	//Canny (not suitable - makes two lines per bone)
	//	//---------------------
	//	cv::Mat canny_edge;
	//	cv::Mat dist_8bit;
	//	grad_dist_xform.convertTo(dist_8bit, CV_8U, 25);
	//	cv::imshow("Dist_8bit", dist_8bit);
	//	cv::Canny(dist_8bit, canny_edge, 9, 3*9);
	//	cv::imshow("Canny", canny_edge);
	//	//---------------------

	//Threshold
	//---------------------
	cv::Mat thresholded;
	cv::threshold(grad_dist_xform,thresholded, 0.7, 255, CV_8U);
	cv::Mat thresh_8bit;
	thresholded.convertTo(thresh_8bit,CV_8U);
	//cv::imshow("Thresh", thresh_8bit);
	//---------------------

	//Erode binary image
	//---------------------
	cv::Mat bin_eroded;
	int erosion_size = 2;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
			cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			cv::Point( erosion_size, erosion_size ) );
	cv::erode(bin_img, bin_eroded, element);
	//cv::imshow("Eroded", bin_eroded);
	//---------------------

	cv::Mat removed_border;
	cv::bitwise_and(thresh_8bit, bin_eroded, removed_border);
	//cv::imshow("removed_border", removed_border);

	cv::Mat thinned = connectivity_preserving_thinning(removed_border);
	//cv::imshow("thinned", thinned);

	cv::Mat dendrites_removed1 = remove_isolated_short_segments(thinned, 5);
	//cv::imshow("dendrites_removed1", dendrites_removed1);

	cv::Mat thinned2 = connectivity_preserving_thinning(dendrites_removed1);
	//cv::imshow("thinned2", thinned2);

	cv::Mat dendrites_removed2 = remove_isolated_short_segments(thinned2, 15);
	cv::imshow("dendrites_removed2", dendrites_removed2);

	//cv::waitKey(80);
	//exit(0);
	return dendrites_removed2;
}

cv::Mat Skeletonization::remove_isolated_short_segments(cv::Mat img_in, int thresh_length)
{
	cv::Mat result = img_in.clone();

	cv::Mat free_nodes(result.rows, result.cols, CV_8U, cv::Scalar(0));
	cv::Mat junction_nodes(result.rows, result.cols, CV_8U, cv::Scalar(0));

	//Make map of end points and junctions:
	for(int row = 1; row < result.rows-1; row++)
	{
		for(int col = 1; col < result.cols-1; col++)
		{
			//Check that pixel is occupied:
			if(result.at<uchar>(row, col) != 0)
			{
				//Get number of neighbours:
				int num_neigbours = 0;
				for(int d_row = -1; d_row <= 1; d_row++)
				{
					for(int d_col = -1; d_col <= 1; d_col++)
					{
						if(d_row == 0 && d_col == 0)
						{
							continue;//(don't count central pixel)
						}
						if(result.at<uchar>(row + d_row, col + d_col) != 0)
						{
							num_neigbours++;
						}
					}
				}
				//Immediately remove isolated pixels:
				if(num_neigbours == 0)
				{
					result.at<uchar>(row, col) = 0;
				}
				//Free node:
				else if(num_neigbours == 1)
				{
					free_nodes.at<uchar>(row, col) = 255;
				}
				//Junction node:
				else if(num_neigbours > 2)
				{
					junction_nodes.at<uchar>(row, col) = 255;
				}

			}
		}
	}

	//DEBUG: Show the nodes images:
	//cv::imshow("Free nodes", free_nodes);
	//cv::imshow("Junction nodes", junction_nodes);

	//For all the free nodes,
	for(int row = 1; row < result.rows-1; row++)
	{
		for(int col = 1; col < result.cols-1; col++)
		{
			if(free_nodes.at<uchar>(row, col) != 0)
			{
				//Walk along until a another free node or a junction node is found
				std::vector<cv::Point> path;
				int walk_row = row;
				int walk_col = col;
				path.push_back(cv::Point(walk_col, walk_row));
				bool next_node_found = false;
				//while(next_node_found == false)
				for(int iter = 0; iter < 100; iter++)
				{
					if(next_node_found == true)
					{
						break;
					}

					bool next_point_found = false;
					for(int d_row = -1; d_row <= 1; d_row++)
					{
						for(int d_col = -1; d_col <= 1; d_col++)
						{
							if( (d_row == 0 && d_col == 0) || //(don't count central pixel)
									//(path[path.size()-1].x == walk_col + d_col
									//&& path[path.size()-1].y == walk_row + d_row) )
									path[path.size()-1] == cv::Point(walk_col + d_col, walk_row + d_row) ||//(don't go where you've just been)
									(path.size()>1 && path[path.size()-2] == cv::Point(walk_col + d_col, walk_row + d_row)) ||
									next_point_found == true)//(don't keep looking if the next point has already been found)
							{
								continue;
							}
							if(result.at<uchar>(walk_row + d_row, walk_col + d_col) != 0)
							{
								walk_row += d_row;
								walk_col += d_col;

								path.push_back(cv::Point(walk_col, walk_row));
								next_point_found = true;

								//Check if another node has been found:
								if((free_nodes.at<uchar>(walk_row, walk_col) != 0 ||
										junction_nodes.at<uchar>(walk_row, walk_col) != 0))
								{
									next_node_found = true;
									//DEBUG:
									if(free_nodes.at<uchar>(walk_row, walk_col) != 0)
										cout << "Found free node" << endl;
									if(junction_nodes.at<uchar>(walk_row, walk_col) != 0)
										cout << "Found junction node" << endl;

									//Delete last element so that breaks are not introduced at junctions
									path.pop_back();
								}

							}
						}
					}
				}
				//DEBUG:

				cout << "(next node " << std::string(next_node_found?"found":"not found") << ") path length = " << path.size() << endl;
				for(int p = 0; p<path.size(); p++)
				{
					cout << path[p] << endl;
				}

				//Check length of path, deleting points if it's shorted than threshold:
				if(path.size() < thresh_length)
				{
					for(int i = 0; i<path.size(); i++)
					{
						result.at<uchar>(path[i].y, path[i].x) = 0;
					}
				}

			}
		}
	}


	//If the distance walked is less than the threshold, delete the node and all points between it and the node found


	return result;
}

cv::Mat Skeletonization::connectivity_preserving_thinning(cv::Mat img_in)
{
	cv::Mat result = img_in.clone();

	//Two iterations should be sufficient (TODO - rather check if result changed between iterations
	bool was_updated_this_iter = true;
	int iter = 0;
	while(was_updated_this_iter == true)
	{
		was_updated_this_iter = false;
		iter++;

		for(int row = 1; row < result.rows-1; row++)
		{
			for(int col = 1; col < result.cols-1; col++)
			{
				//Check that pixel is occupied:
				if(result.at<uchar>(row, col) != 0)
				{
					//Establish original connectivity:
					//(it is trivial that all 'on' pixels are known to be connected each other
					//when the central pixel is 'on')

					//Establish connectivity if removed:
					//Check that number of connected components is still 1:
					int seed_d_row = -2;
					int seed_d_col = -2;
					bool found_seed = false;
					for(int d_row = -1; d_row <= 1; d_row++)
					{
						if(found_seed)
						{
							break;
						}
						for(int d_col = -1; d_col <= 1; d_col++)
						{
							if(d_row == 0 && d_col == 0)
							{
								continue;
							}
							if(result.at<uchar>(row + d_row, col + d_col) != 0)
							{
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

					if(found_seed)
					{
						cv::Mat propergated(3,3, CV_8U, cv::Scalar(0));

						for(int s_row = -1; s_row <= 1; s_row++)
						{
							for(int s_col = -1; s_col <= 1; s_col++)
							{
								if(result.at<uchar>(row + s_row, col + s_col) != 0)
								{
									num_in_win++;
								}
							}
						}

						//Set seed pixel to 'on': (adding (1,1) offset to get beteen (-1,-1) and (0,0) origin)
						propergated.at<uchar>(seed_d_row+1, seed_d_col+1) = 255;

						//Iterate thrice to cover the 3x3 region:
						for(int iter = 0; iter < 3; iter++)
						{
							//Propagate using 8-connectivity from seed:
							for(int s_row = 0; s_row < 3; s_row++)
							{
								for(int s_col = 0; s_col < 3; s_col++)
								{
									if(propergated.at<uchar>(s_row, s_col) != 0)
									{
										for(int t_d_row = -1; t_d_row <= 1; t_d_row++)
										{
											for(int t_d_col = -1; t_d_col <= 1; t_d_col++)
											{
												if((s_row + t_d_row >= 0 && s_row + t_d_row < 3
														&& s_col + t_d_col >= 0 && s_col + t_d_col < 3)//check bounds
														&& result.at<uchar>(row + s_row + t_d_row-1, col + s_col + t_d_col-1) != 0)
												{
													propergated.at<uchar>(s_row + t_d_row, s_col + t_d_col) = 255;
												}
											}
										}
									}

								}
							}
						}
						for(int s_row = 0; s_row < 3; s_row++)
						{
							for(int s_col = 0; s_col < 3; s_col++)
							{
								if(propergated.at<uchar>(s_row, s_col) != 0)
								{
									num_con++;
								}
							}
						}
					}

					//Set on again if neighbours get disconnected, or if it has fewer than 2 neighours
					//(in which case it is the end of a line, which must not be shortened):
					if(num_con != num_in_win || num_con < 2)
					{
						//Set central pixel back to 'on':
						result.at<uchar>(row, col) = 255;

						//DEBUG:
						//cout << "num_con: " << num_con << ", num_in_win: " << num_in_win << endl;
					}
					else
					{
						was_updated_this_iter = true;
					}

				}
			}
		}

	}

	//DEBUG:
	cout << "Skeleton thinning done in " << iter << " iterations" << endl;

	return result;
}

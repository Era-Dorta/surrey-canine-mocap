/*
 * PixelSearch.cpp
 *
 *  Created on: 17 Jan 2014
 *      Author: m04701
 */

#include "PixelSearch.h"

//Given a uchar 1 channel image, and a start points it returns true and a position
//if it finds a white pixel next to it. False otherwise.
bool PixelSearch::get_neighbor_white_pixel(cv::Mat& img, int i_row, int i_col,
		int &res_row, int &res_col) {
	bool go_top, go_bot, go_left, go_right;
	//TODO Maybe it would be better to calculate this as they are needed
	//and not all together in the beggining
	go_top = i_row > 0;
	go_bot = i_col < img.rows - 1;
	go_left = i_col > 0;
	go_right = i_col < img.cols - 1;

	//Search order is
	// 1 0 2
	// 3 x 4
	// 6 5 7
	//This is done since bone merging starts from the bottom, so we want to give
	//priority to follow the path of the bone upwards. Left over right is an
	//arbitrary decision
	if (go_top && (int) img.at<uchar>(i_row - 1, i_col) == 255) {
		res_row = i_row - 1;
		res_col = i_col;
		return true;
	} else if (go_top && go_left
			&& (int) img.at<uchar>(i_row - 1, i_col - 1) == 255) {
		res_row = i_row - 1;
		res_col = i_col - 1;
		return true;
	} else if (go_top && go_right
			&& (int) img.at<uchar>(i_row - 1, i_col + 1) == 255) {
		res_row = i_row - 1;
		res_col = i_col + 1;
		return true;
	} else if (go_left && (int) img.at<uchar>(i_row, i_col - 1) == 255) {
		res_row = i_row;
		res_col = i_col - 1;
		return true;
	} else if (go_right && (int) img.at<uchar>(i_row, i_col + 1) == 255) {
		res_row = i_row;
		res_col = i_col + 1;
		return true;
	} else if (go_bot && (int) img.at<uchar>(i_row + 1, i_col) == 255) {
		res_row = i_row + 1;
		res_col = i_col;
		return true;
	} else if (go_bot && go_left
			&& (int) img.at<uchar>(i_row + 1, i_col - 1) == 255) {
		res_row = i_row + 1;
		res_col = i_col - 1;
		return true;
	} else if (go_bot && go_right
			&& (int) img.at<uchar>(i_row + 1, i_col + 1) == 255) {
		res_row = i_row + 1;
		res_col = i_col + 1;
		return true;
	}
	return false;
}

bool PixelSearch::get_bottom_white_pixel(cv::Mat& img, int &res_row,
		int &res_col) {
	return get_bottom_white_pixel(img, res_row, res_col, img.rows - 1, 0);
}

bool PixelSearch::get_bottom_white_pixel(cv::Mat& img, int &res_row,
		int &res_col, int i_row, int i_col) {
	//Since we want the bottom-left white pixel
	//and 0,0 is top-left in openCV
	for (int row = i_row; row > 0; row--) {
		for (int col = i_col; col < img.cols; col++) {
			//If pixel is white
			if ((int) img.at<uchar>(row, col) == 255) {
				res_row = row;
				res_col = col;
				return true;
			}
		}
	}
	return false;
}

bool PixelSearch::get_top_left_white_pixel(const cv::Mat& img, int i_row,
		int i_col, int &res_row, int &res_col) {
	bool go_top, go_left, go_bot;

	go_top = i_row > 0;
	go_left = i_col > 0;
	go_bot = i_col < img.rows - 1;

	//Search order is
	// 1 0
	// 2 x
	// 3
	if (go_top && (int) img.at<uchar>(i_row - 1, i_col) == 255) {
		res_row = i_row - 1;
		res_col = i_col;
		return true;
	} else if (go_top && go_left
			&& (int) img.at<uchar>(i_row - 1, i_col - 1) == 255) {
		res_row = i_row - 1;
		res_col = i_col - 1;
		return true;
	} else if (go_left && (int) img.at<uchar>(i_row, i_col - 1) == 255) {
		res_row = i_row;
		res_col = i_col - 1;
		return true;
	} else if (go_bot && go_left
			&& (int) img.at<uchar>(i_row + 1, i_col - 1) == 255) {
		res_row = i_row + 1;
		res_col = i_col - 1;
		return true;
	}
	return false;
}

bool PixelSearch::get_nearest_white_pixel(const cv::Mat& img, int i_row,
		int i_col, int& res_row, int& res_col) {
	int increment = 1;
	int num_elements = 2;
	int aux_num_elements = 2;
	int i;
	int total = img.cols * img.rows;
	int current = 1;

	//To search the nearest white pixel this method goes in squares
	//searching, it will not give the closer one, but is a good approximation

	// 0 1 2
	// 7 x 3
	// 6 5 4

	//If it does not find it, then is goes to a bigger outer square and so on
	while (current < total) {

		//Right
		res_row = i_row - increment;
		if (res_row >= 0) {
			res_col = i_col - increment;
			if (res_col < 0) {
				num_elements = num_elements + res_col;
				res_col = 0;
			}

			i = 0;
			while (res_col < img.cols && i < num_elements) {
				if (img.at<uchar>(res_row, res_col) == 255) {
					return true;
				}
				res_col++;
				i++;
			}
			current += i;
		}

		//Bottom
		num_elements = aux_num_elements;
		res_col = i_col + increment;
		if (res_col < img.cols) {
			res_row = i_row - increment;
			if (res_row < 0) {
				num_elements = num_elements + res_row;
				res_row = 0;
			}

			i = 0;
			while (res_row < img.rows && i < num_elements) {
				if (img.at<uchar>(res_row, res_col) == 255) {
					return true;
				}
				res_row++;
				i++;
			}
			current += i;
		}

		//Left
		num_elements = aux_num_elements;
		res_row = i_row + increment;
		if (res_row < img.rows) {
			res_col = i_col + increment;
			if (res_col >= img.cols) {
				num_elements = num_elements - (res_col - img.cols + 1);
				res_col = img.cols - 1;
			}

			i = 0;
			while (res_col >= 0 && i < num_elements) {
				if (img.at<uchar>(res_row, res_col) == 255) {
					return true;
				}
				res_col -= 1;
				i++;
			}
			current += i;
		}

		//Up
		num_elements = aux_num_elements;
		res_col = i_col - increment;
		if (res_col >= 0) {
			res_row = i_row + increment;
			if (res_row > img.rows) {
				num_elements = num_elements - (res_row - img.rows + 1);
				res_row = img.rows - 1;
			}

			i = 0;
			while (res_row >= 0 && i < num_elements) {
				if (img.at<uchar>(res_row, res_col) == 255) {
					return true;
				}
				res_row -= 1;
				i++;
			}
			current += i;
		}

		increment++;
		aux_num_elements += 2;
		num_elements = aux_num_elements;
	}

	return false;
}

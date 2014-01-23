/*
 * PixelSearch.h
 *
 *  Created on: 17 Jan 2014
 *      Author: m04701
 */

#ifndef PIXELSEARCH_H_
#define PIXELSEARCH_H_

#include "opencv2/opencv.hpp"

namespace PixelSearch {
	//Given a uchar 1 channel image, and a start points it returns true and a position
	//if it finds a white pixel next to it. False otherwise.
	bool get_neighbor_white_pixel(cv::Mat& img, int i_row, int i_col,
			int &res_row, int &res_col);

	//Auxiliary method that finds the white pixel situated most at the
	//bottom left of the image.
	bool get_bottom_white_pixel(cv::Mat& img, int &res_row, int &res_col);

	bool get_bottom_white_pixel(cv::Mat& img, int &res_row, int &res_col,
			int i_row, int i_col);

	bool get_top_left_white_pixel(const cv::Mat& img, int i_row, int i_col,
			int &res_row, int &res_col);

	bool get_nearest_white_pixel(const cv::Mat& img, int i_row, int i_col,
			int &res_row, int &res_col);

	void cascade_down_lower_pixel(const cv::Mat& img, int i_row, int i_col,
			int &res_row, int &res_col);
}

#endif /* PIXELSEARCH_H_ */

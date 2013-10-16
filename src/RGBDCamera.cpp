/*
 * RGBDCamera.cpp
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#include "RGBDCamera.h"

using std::cout;
using std::endl;

cv::Mat remove_isolated_short_segments(cv::Mat img_in, int thresh_length)
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

cv::Mat connectivity_preserving_thinning(cv::Mat img_in)
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

void dist_transform_skeletonization(cv::Mat& seg_img)
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

	cv::imshow("Binary image", bin_img);

	//---------------------

	//Perform distance transform:
	//---------------------
	cv::Mat dist_transform_img;

	cv::distanceTransform(bin_img, dist_transform_img, CV_DIST_L2, CV_DIST_MASK_PRECISE);

	cv::imshow("Dist transform", dist_transform_img/100.f);

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

	cv::imshow("2nd deriv dist transform", grad_dist_xform/10);

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
	cv::imshow("removed_border", removed_border);

	cv::Mat thinned = connectivity_preserving_thinning(removed_border);
	cv::imshow("thinned", thinned);

	cv::Mat dendrites_removed1 = remove_isolated_short_segments(thinned, 5);
	cv::imshow("dendrites_removed1", dendrites_removed1);

	cv::Mat thinned2 = connectivity_preserving_thinning(dendrites_removed1);
	cv::imshow("thinned2", thinned2);

	cv::Mat dendrites_removed2 = remove_isolated_short_segments(thinned2, 15);
	cv::imshow("dendrites_removed2", dendrites_removed2);

	cv::waitKey(80);
	//exit(0);
}


RGBD_Camera::RGBD_Camera(std::string dataset_path, std::string cam_name):
		_dataset_path(dataset_path), _cam_name(cam_name),
		cam_group(new osg::Group),
		skel_vis_group(new osg::Group),
		cam_pose_xform(new osg::MatrixTransform)
{
	cout << "Loading " << _cam_name << endl;

	//Load intrinsic and extrinsic calibration:
	load_calibration();

	//Load the timestamp files:
	std::string ts_depth_dev_fn(_dataset_path + "/" + _cam_name + "/Timestamp_Depth/Depth_timestamps.txt");
	load_timestamps(ts_depth_dev, ts_depth_dev_fn);
	std::string ts_depth_global_fn(_dataset_path + "/" + _cam_name + "/Timestamp_Depth/Depth_tick_timestamps.txt");
	load_timestamps(ts_depth_global, ts_depth_global_fn);
	std::string ts_rgb_dev_fn(_dataset_path + "/" + _cam_name + "/Timestamp_RGB/RGB_timestamps.txt");
	load_timestamps(ts_rgb_dev, ts_rgb_dev_fn);
	std::string ts_rgb_global_fn(_dataset_path + "/" + _cam_name + "/Timestamp_RGB/RGB_tick_timestamps.txt");
	load_timestamps(ts_rgb_global, ts_rgb_global_fn);

	//	//DEBUG:
	//	cout << "ts_rgb_dev[45]: " << (uint64)ts_rgb_dev[45] << endl;
	//	cout << "ts_rgb_global[45]: " << (uint64)ts_rgb_global[45] << endl;
	//	cout << "ts_depth_dev[45]: " << (uint64)ts_depth_dev[45] << endl;
	//	cout << "ts_depth_global[45]: " << (uint64)ts_depth_global[45] << endl;

	//Get list of RGB frames:
	std::vector<std::string> rgb_files;
	get_file_names(_dataset_path + "/" + cam_name + "/RGB", &rgb_files);

	cout << "Found " << rgb_files.size() << " images." << endl;

	//Parse the RGB filenames:
	for(std::vector<std::string>::iterator i(rgb_files.begin()); i != rgb_files.end(); ++i)
		//for(std::vector<std::string>::iterator i(rgb_files.begin()); i != rgb_files.begin()+10; ++i)
	{
		int frame_num = boost::lexical_cast<int>(((*i).substr((*i).length()-9,5)).c_str());

		load_frame(frame_num);

	}

	//Set camera image size according to the first image:
	d_rows = frames.begin()->second.depth_img.rows;
	d_cols = frames.begin()->second.depth_img.cols;

	//Segment the frames (remove the background):
	segment_frames();

	//TODO This produces much betters results but is quite slow
	//bilateral_filter_frames();

	//DEBUG TEST: Distance transform skeletonization
	if(cam_name == "Cam_02")
	{
		for(int i = 90; i < 120; i++)
		{
		dist_transform_skeletonization(frames[i].depth_img);
		cv::waitKey(0);
		}
	}

	//	//Pre-compute the surface paths for all frames:
	//	cout << "Computing surface paths" << endl;
	//	for(int i = frames.begin()->first; i < frames.end()->first; i++)
	//	{
	//		std::vector< std::vector<float3> > surface_paths_3d;
	//		get_surface_paths_3d(i, surface_paths_3d);
	//		frame_surface_paths_3d[i] = surface_paths_3d;
	//	}


	create_cam_geom();

}

RGBD_Camera::~RGBD_Camera()
{
	// TODO Auto-generated destructor stub
}

void RGBD_Camera::load_timestamps(std::map<int, double>& ts, std::string fn)
{
	std::ifstream file;
	file.open(fn.c_str());
	int frame_num(-1);
	double val(-69);
	if(file.is_open())
	{
		file >> frame_num;
		file >> val;
		ts[frame_num] = val;
		while(!file.eof())
		{
			file >> frame_num;
			file >> val;
			ts[frame_num] = val;
		}

		file.close();
	}
	else
	{
		cout << "Error: Failed to open timestamp file " << fn << endl;
	}

}

void RGBD_Camera::load_frame(int frame_num)
{
	RGBD_Frame frame;

	frame.frame_num = frame_num;

	//Read images:
	char fn[1024];
	sprintf(fn, "%s/%s/RGB/%05d.png", _dataset_path.c_str(), _cam_name.c_str(), frame_num);
	frame.rgb_img = cv::imread(fn,-1);
	if(frame.rgb_img.data == NULL)
	{
		cout << "Error: failed to read image " << fn << endl;
		exit(-1);
	}

	sprintf(fn, "%s/%s/Depth/%05d.png", _dataset_path.c_str(), _cam_name.c_str(), frame_num);
	frame.depth_img = cv::imread(fn,-1);
	if(frame.depth_img.data == NULL)
	{
		cout << "Error: failed to read image " << fn << endl;
		exit(-1);
	}

	frames[frame_num] = frame;

}

void RGBD_Camera::load_calibration(void)
{
	std::string K_fn(_dataset_path + "/" + _cam_name + "/Calibration/K_rgb.txt");
	std::string T_fn(_dataset_path + "/" + _cam_name + "/Calibration/T_rgb.txt");

	//Intrinsics:
	//------------------------------------
	std::ifstream K_file;
	K_file.open(K_fn.c_str());
	float K_val[9];
	if(K_file.is_open())
	{
		for(int i=0; i<9; i++)
		{
			K_file >> K_val[i];
		}

		K_file.close();
	}
	else
	{
		cout << "Error: Failed to open intrinsic calibration file" << endl;
		exit(-1);
	}
	//NB! Cloning because the array with the data will go out of scope when the function returns:
	cv::Mat K = cv::Mat(3, 3, CV_32F, K_val, 3*sizeof(float)).clone();
	K_rgb = K;

	//DEBUG:
	//cout << "Intrinsics read from file: " << K << endl;

	//------------------------------------

	//Extrinsics:
	//------------------------------------
	std::ifstream T_file;
	T_file.open(T_fn.c_str());
	float T_val[16];
	if(T_file.is_open())
	{
		for(int i=0; i<16; i++)
		{
			T_file >> T_val[i];
		}

		T_file.close();
	}
	else
	{
		cout << "Error: Failed to open extrinsic calibration file" << endl;
		exit(-1);
	}
	//NB! Cloning because the array with the data will go out of scope when the function returns:
	cv::Mat T = cv::Mat(4, 4, CV_32F, T_val, 4*sizeof(float)).clone();
	T_rgb = T;

	//DEBUG:
	//cout << "Extrinsics read from file: " << T << endl;

	//------------------------------------

}

int RGBD_Camera::get_first_frame_num(void)
{
	int result = frames.begin()->first;

	//DEBUG:
	//cout << "first_frame: " << result << endl;

	return result;


}

int RGBD_Camera::get_last_frame_num(void)
{
	int result = frames.rbegin()->first;

	//DEBUG:
	//cout << "last_frame: " << result << endl;

	return result;
}

void RGBD_Camera::create_cam_geom(void)
{

	//Set camera visualization colour:
	//-----------------
	int cam_num = boost::lexical_cast<int>(_cam_name.substr(_cam_name.length()-2,2).c_str());
	vis_colour = hsv_2_rgb(osg::Vec3(360*(cam_num/5.f), 0.6, 0.7));
	//-----------------

	//Camera pose matrix:
	//-----------------
	//osg::ref_ptr<osg::MatrixTransform> cam_pose_xform(new osg::MatrixTransform);
	osg::Matrix cam_pose_mat = cvmat4x4_2_osgmat(T_rgb);
	cam_pose_xform->setMatrix(cam_pose_mat);
	//-----------------

	//Camera icon:
	//-----------------
	osg::ref_ptr<osg::Geode> cam_icon = create_cam_icon(vis_colour);
	cam_pose_xform->addChild(cam_icon);
	//-----------------

	//Camera axes:
	//-----------------
	osg::ref_ptr<osg::Geode> cam_axes = create_axes();
	osg::ref_ptr<osg::MatrixTransform> half_size(new osg::MatrixTransform);
	osg::Matrix half_sz = osg::Matrix::scale(0.7,0.7,0.7);
	half_size->setMatrix(half_sz);
	half_size->addChild(cam_axes);
	cam_pose_xform->addChild(half_size);
	//-----------------

	//Camera label:
	//-----------------
	osg::ref_ptr<osg::Group> cam_label = create_3D_label(_cam_name, osg::Vec3(0,0.2,0), vis_colour);
	cam_pose_xform->addChild(cam_label);
	//-----------------

	//Depth map geometry:
	//-----------------
	//Initialize depth polygonizer:
	//depth_poly.init(d_rows, d_cols, vis_colour);
	depth_surf.init(d_rows, d_cols, vis_colour);

	//cam_pose_xform->addChild(depth_poly.depth_poly_geode);
	cam_pose_xform->addChild(depth_surf.depth_surfel_geode);
	//-----------------

	//Skeleton visualization geometry:
	//-----------------
	cam_pose_xform->addChild(skel_vis_group);
	//-----------------

	cam_group->addChild(cam_pose_xform);

}

osg::Geode* RGBD_Camera::create_cam_icon(osg::Vec3 vis_colour)
{

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(5);
	(*vertices)[0].set(osg::Vec3(0,				0,		0));
	(*vertices)[1].set(osg::Vec3(-0.0561,	0.0421,	0.1));
	(*vertices)[2].set(osg::Vec3(-0.0561,  -0.0421,	0.1));
	(*vertices)[3].set(osg::Vec3( 0.0561,  -0.0421,	0.1));
	(*vertices)[4].set(osg::Vec3( 0.0561,	0.0421,	0.1));
	osg::ref_ptr<osg::DrawElementsUInt> indices =
			new osg::DrawElementsUInt(GL_LINES, 16);
	(*indices)[0] = 4; (*indices)[1] = 1;
	(*indices)[2] = 1; (*indices)[3] = 2;
	(*indices)[4] = 2; (*indices)[5] = 3;
	(*indices)[6] = 3; (*indices)[7] = 4;

	(*indices)[8]  = 0; (*indices)[9]  = 1;
	(*indices)[10] = 0; (*indices)[11] = 2;
	(*indices)[12] = 0; (*indices)[13] = 3;
	(*indices)[14] = 0; (*indices)[15] = 4;

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray( vertices.get() );
	geom->addPrimitiveSet( indices.get() );

	osg::ref_ptr<osg::Geode> cam_vis = new osg::Geode;
	cam_vis->addDrawable(geom.get());

	//Wireframe:
	osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	cam_vis->getOrCreateStateSet()->setAttribute( pm.get() );

	//Turn off lighting:
	cam_vis->getOrCreateStateSet()->setMode( GL_LIGHTING,
			osg::StateAttribute::OFF |osg::StateAttribute::OVERRIDE);

	//Set colour:
	osg::ref_ptr<osg::Vec4Array> colours = new osg::Vec4Array;
	colours->push_back( osg::Vec4(vis_colour, 1.0f) );
	geom->setColorArray(colours.get());
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	//Make the lines thicker:
	//-------------
	osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth();
	linewidth->setWidth(2.0f);
	geom->getOrCreateStateSet()->setAttributeAndModes(linewidth,
			osg::StateAttribute::ON);
	//-------------

	return cam_vis.release();
}

cv::Mat RGBD_Camera::get_K_rgb(void)
{
	return K_rgb;
}

cv::Mat RGBD_Camera::get_T_rgb(void)
{
	return T_rgb;
}

cv::Mat* RGBD_Camera::get_depth_map(int frame_num)
{
	if(frames.find(frame_num) != frames.end())
	{
		return &frames[frame_num].depth_img;
	}
	else
	{
		//Load the frame from disk:
		load_frame(frame_num);
		if(frames.find(frame_num) != frames.end())
		{
			return &frames[frame_num].depth_img;
		}
		else
		{
			cout << "Error: frame " << frame_num << " does not exist in the data.";
			exit(-1);
		}
	}
}

cv::Mat* RGBD_Camera::get_rgb_image(int frame_num)
{
	if(frames.find(frame_num) != frames.end())
	{
		return &frames[frame_num].rgb_img;
	}
	else
	{
		//Load the frame from disk:
		load_frame(frame_num);
		if(frames.find(frame_num) != frames.end())
		{
			return &frames[frame_num].depth_img;
		}
		else
		{
			cout << "Error: frame " << frame_num << " does not exist in the data.";
			exit(-1);
		}
	}
}

std::string RGBD_Camera::get_cam_name(void)
{
	return _cam_name;
}

int RGBD_Camera::get_d_rows(void)
{
	return d_rows;
}

int RGBD_Camera::get_d_cols(void)
{
	return d_cols;
}

osg::Vec3 RGBD_Camera::get_vis_colour(void)
{
	return vis_colour;
}

void RGBD_Camera::segment_frames(void)
{
	//Get a slightly noise reduced and more complete background plate by averaging the first few 10s of frames:
	//--------------------------------
	cv::Mat background_plate_f(d_rows, d_cols, CV_32F, 0.f);
	cv::Mat background_weight(cv::Mat::zeros(d_rows, d_cols, CV_16U));

	for(std::map<int, RGBD_Frame>::iterator i(frames.begin()); i->first < frames.begin()->first + 60; ++i)
	{
		//Incremental averaging valid pixels:
		for(int row = 0; row < d_rows; row++)
		{
			for(int col = 0; col < d_cols; col++)
			{
				if((*i).second.depth_img.at<ushort>(row,col) != 0)
				{
					float w_old = (float)background_weight.at<ushort>(row,col);
					background_plate_f.at<float>(row,col) =
							(background_plate_f.at<float>(row,col)*w_old +
									(*i).second.depth_img.at<ushort>(row,col)*1.0f)/(w_old+1);
					background_weight.at<ushort>(row,col)++;
				}
			}
		}
	}

	//DEBUG:
	//cv::imshow("BG_weight", background_weight*64000/60);
	//cv::waitKey(0);

	//Convert float image to ushort:
	cv::Mat background_plate(d_rows, d_cols, CV_16U, 0);
	background_plate_f.convertTo(background_plate, CV_16U);
	//--------------------------------



	//Set all depth pixels that are consistent (within the noise) with the background plate to zero:
	//--------------------------------

	for(std::map<int, RGBD_Frame>::iterator i(frames.begin()); i != frames.end(); ++i)
	{
		//Set pixels to zero if they are within the (depth dependent) noise floor of the background plate::
		for(int row = 0; row < d_rows; row++)
		{
			for(int col = 0; col < d_cols; col++)
			{

				float3 global_pt = global_coord((*i).first, row, col);

				//Noise floor is depth dependent quantization step times an extra tolerance factor of 4 to get more filtering:
				float noise_floor = 4*(2.8e-6*(background_plate.at<ushort>(row,col))*(background_plate.at<ushort>(row,col)));
				if(!((abs((int)background_plate.at<ushort>(row,col) - (int)(*i).second.depth_img.at<ushort>(row,col)) > noise_floor ) &&
						global_pt.x > -2.5 &&//And bounding box segmentation
						global_pt.x < 2.5 &&
						//global_pt.y < 2.5 &&
						//global_pt.y < 2.5 &&
						global_pt.z > 0.6 &&
						global_pt.z < 1.4))
				{
					//Point is classifies as background, set it to invalid (zero):
					(*i).second.depth_img.at<ushort>(row,col) = 0;
				}
			}
		}
	}

	//--------------------------------

}

void RGBD_Camera::get_surface_paths(int frame_num, std::vector< std::vector<cv::Point> >& surface_paths)
{
	//cout << "Computing surface paths" << endl;

	//std::vector< std::vector<cv::Point> > surface_paths;

	//Loop from bottom to top:
	for(int row = d_rows-1; row>=0; row--)
	{

		std::vector<int> mid_cols;
		int col = 0;
		while(col < d_cols)
		{
			if(frames[frame_num].depth_img.at<ushort>(row, col) != 0)
			{
				int begin_col = col;
				int thresh = 30;//30mm - TODO make depth dependent
				if(col < d_cols - 1)
					col++;
				while(abs((int)frames[frame_num].depth_img.at<ushort>(row, col) -
						(int)frames[frame_num].depth_img.at<ushort>(row, col-1)) < thresh)
				{
					if(col < d_cols - 1)
						col++;
					else
						break;
				}
				int end_col = col;
				int mid_col = (end_col+begin_col)/2;//(rounding to integer)
				mid_cols.push_back(mid_col);

				//DEBUG: Show midpoint
				//d_img.at<ushort>(row, mid_col) = 5000;

				col++;
			}
			else
			{
				col++;
			}
		}

		//Append each detected midpoint from this row to a existing path (if present) otherwise begin a new one:

		for(int mc = 0; mc<mid_cols.size(); mc++)
		{
			bool matched_to_existing = false;

			//See if possible to append to existing contour:
			for(int sp = 0; sp < surface_paths.size(); sp++)
			{
				cv::Point last_pt(*(surface_paths[sp].end()-1));
				if(cont_path(frames[frame_num].depth_img,
						last_pt, cv::Point(mid_cols[mc], row)) == true)
				{
					matched_to_existing = true;
					surface_paths[sp].push_back(cv::Point(mid_cols[mc], row));
				}
			}
			//Or start a new path:
			if(matched_to_existing == false)
			{
				std::vector<cv::Point> new_path;
				new_path.push_back(cv::Point(mid_cols[mc], row));
				surface_paths.push_back(new_path);
			}
		}

	}

	//Delete paths with fewer than min_length elements because they're probably noise:
	int min_length = 25;
	std::vector<std::vector<cv::Point> >::iterator sp(surface_paths.begin());

	while(sp != surface_paths.end())
	{
		if((*sp).size() < min_length)
		{
			surface_paths.erase(sp);
		}
		else
		{
			sp++;
		}
	}

	//	//DEBUG:
	//	//Draw the paths in colour and display:
	//	//-----------------------------
	//	cv::Mat path_vis;
	//	frames[frame_num].depth_img.convertTo(path_vis, CV_8UC3, 1./16, 0);
	//	cv::cvtColor(path_vis, path_vis, CV_GRAY2BGR,3);
	//
	//	for(int sp = 0; sp < surface_paths.size(); sp++)
	//	{
	//
	//		cout << "Path " << sp << ", " << surface_paths[sp].size() << " elements" << endl;
	//
	//		osg::Vec3 vis_col = hsv_2_rgb(osg::Vec3(360*(sp/11.f), 0.6, 0.7));
	//		cv::Scalar vc(255*vis_col.z(), 255*vis_col.y(), 255*vis_col.x());
	//
	//		for(int element = 0; element < surface_paths[sp].size()-1; element++)
	//		{
	//			cv::line(path_vis,
	//					cv::Point(surface_paths[sp][element].x, surface_paths[sp][element].y),
	//					cv::Point(surface_paths[sp][element+1].x, surface_paths[sp][element+1].y), vc);
	//		}
	//	}
	//	cv::imshow("Surface paths", path_vis);
	//	cv::waitKey(10);
	//	frames[frame_num].depth_img.convertTo(path_vis, CV_8UC3, 1./16, 0);
	//	cv::cvtColor(path_vis, path_vis, CV_GRAY2BGR,3);
	//	//-----------------------------

}

bool RGBD_Camera::cont_path(cv::Mat& d_img, cv::Point last, cv::Point current)
{

	int thresh = 30;//30mm

	//Does point a come the row below point b?
	if(last.y != current.y + 1)
	{
		//DEBUG:
		//cout << "Last y: " << last.y << ", Current y: " << current.y << endl;

		return false;
	}

	//Is there a continuous path between them?
	cv::Point left;
	cv::Point right;
	if(last.x > current.x)
	{
		left = current;
		right = last;
	}
	else
	{
		right = current;
		left = last;
	}

	//If points directly above one another:
	if(left.x == right.x)
	{
		if(d_img.at<ushort>(left.y, left.x) != 0 &&
				abs((int)d_img.at<ushort>(right.y, right.x) - (int)d_img.at<ushort>(left.y, left.x)) < thresh)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	//Check lower and upper horizontal lines for self consistency and consistency with each other:
	for(int i = left.x; i < right.x-1; i++)
	{
		if(!(d_img.at<ushort>(left.y, i) != 0 &&
				abs((int)d_img.at<ushort>(left.y, i) - (int)d_img.at<ushort>(left.y, i+1)) < thresh &&// horizontal consistency
				abs((int)d_img.at<ushort>(right.y, i) - (int)d_img.at<ushort>(right.y, i+1)) < thresh))// "
		{
			return false;
		}
	}
	for(int i = left.x; i < right.x; i++)
	{
		if(!(abs((int)d_img.at<ushort>(left.y, i) - (int)d_img.at<ushort>(right.y, i)) < thresh))//vertical consistency
		{
			return false;
		}
	}

	//	//DEBUG:
	//	if(abs((int)(d_img.at<ushort>(right.y, right.x)) - (int)(d_img.at<ushort>(left.y, left.x))) > thresh)
	//	{
	//		cout << "abs(right.z - left.z) " << abs((int)(d_img.at<ushort>(right.y, right.x)) - (int)(d_img.at<ushort>(left.y, left.x))) <<
	//				", right " << right  << ", left " << left << endl;
	//		//exit(-1);
	//	}


	return true;
}

void RGBD_Camera::get_surface_paths_3d(int frame_num, std::vector< std::vector<float3> >& surface_paths_3d)
{

	//Timer t_2d("2d");

	std::vector< std::vector<cv::Point> > surface_paths_2d;
	get_surface_paths(frame_num, surface_paths_2d);

	//t_2d.tock_print();

	//Timer t_3d("3d");

	float3x3 K_d_f3x3;
	for(int i = 0; i<9; i++)
		K_d_f3x3.val[i] = K_rgb.at<float>(i);

	surface_paths_3d.clear();
	for(int sp = 0; sp < surface_paths_2d.size(); sp++)
	{
		std::vector<float3> temp(surface_paths_2d[sp].size());
		surface_paths_3d.push_back(temp);

		for(int element = 0; element < surface_paths_3d[sp].size(); element++)
		{
			int row = surface_paths_2d[sp][element].y;
			int col = surface_paths_2d[sp][element].x;

			//Read depth pixel (converted from mm to m):
			float depth = (((ushort*)(frames[frame_num].depth_img.data))[row*frames[frame_num].depth_img.step1() + col])/1000.f;
			//Reproject it:
			float3 depth_pix_hom = make_float3(col, row, 1.f);
			float3 vert = depth*((K_d_f3x3.inverse())*depth_pix_hom);
			//Add to array (negate y and z to correct orientation)://11/07/2013 Update - don't negate
			surface_paths_3d[sp][element] = vert;

			//			//DEBUG:
			//			if(element > 0)
			//			{
			//				if( fabs(surface_paths_3d[sp][element].z - surface_paths_3d[sp][element-1].z) > 30e-3)
			//				{
			//					cout << "fabs(current - prev) " << fabs(surface_paths_3d[sp][element].z - surface_paths_3d[sp][element-1].z) << endl;
			//				}
			//			}

		}
	}

	//t_3d.tock_print();
}

float3 RGBD_Camera::global_coord(int frame_num, int row, int col)
{

	float3x3 K_d_f3x3;
	for(int i = 0; i<9; i++)
		K_d_f3x3.val[i] = K_rgb.at<float>(i);

	//Reproject it:
	float depth = ((float)frames[frame_num].depth_img.at<ushort>(row, col))/1000.f;
	float3 depth_pix_hom = make_float3(col, row, 1.f);
	float3 local = depth*((K_d_f3x3.inverse())*depth_pix_hom);

	float4 local_hom = make_float4(local, 1.f);

	//Convert cv mat to float4x4
	float4x4 T_rgb_float4x4;
	for(int r = 0; r<4; r++)
	{
		for(int c = 0; c<4; c++)
		{
			T_rgb_float4x4.val[r*4 + c] = ((float*)T_rgb.data)[r*4 + c];
		}
	}

	float4 result_hom = T_rgb_float4x4*local_hom;

	float3 result = make_float3(result_hom);

	return result;
}

void RGBD_Camera::bilateral_filter_frames(void)
{
	cout << "Bilateral filtering frames" << endl;

	for(std::map<int, RGBD_Frame>::iterator i(frames.begin()); i != frames.end(); ++i)
	{
		cv::Mat depth_map_float;
		(*i).second.depth_img.convertTo(depth_map_float, CV_32F);
		cv::Mat depth_bilateral_filtered;
		cv::bilateralFilter(depth_map_float, depth_bilateral_filtered,-1, 25, 2);
		depth_bilateral_filtered.convertTo(((*i).second.depth_img), CV_16U);
	}
}

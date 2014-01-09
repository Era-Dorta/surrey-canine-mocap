/*
 * RGBDCamera.cpp
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#include "RGBDCamera.h"

using std::cout;
using std::endl;

RGBD_Camera::RGBD_Camera(std::string dataset_path, std::string cam_name) :
			_dataset_path(dataset_path), _cam_name(cam_name),
			cam_group(new osg::Group), skel_vis_group(new osg::Group),
			cam_pose_xform(new osg::MatrixTransform) {
	cout << "Loading " << _cam_name << endl;

	//Load intrinsic and extrinsic calibration:
	load_calibration();

	//Load the timestamp files:
	std::string ts_depth_dev_fn(
			_dataset_path + "/" + _cam_name
					+ "/Timestamp_Depth/Depth_timestamps.txt");
	load_timestamps(ts_depth_dev, ts_depth_dev_fn);
	std::string ts_depth_global_fn(
			_dataset_path + "/" + _cam_name
					+ "/Timestamp_Depth/Depth_tick_timestamps.txt");
	load_timestamps(ts_depth_global, ts_depth_global_fn);
	std::string ts_rgb_dev_fn(
			_dataset_path + "/" + _cam_name
					+ "/Timestamp_RGB/RGB_timestamps.txt");
	load_timestamps(ts_rgb_dev, ts_rgb_dev_fn);
	std::string ts_rgb_global_fn(
			_dataset_path + "/" + _cam_name
					+ "/Timestamp_RGB/RGB_tick_timestamps.txt");
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
	for (std::vector<std::string>::iterator i(rgb_files.begin() + 90);
			i != rgb_files.begin() + 150; ++i)
			//for(std::vector<std::string>::iterator i(rgb_files.begin()); i != rgb_files.begin()+10; ++i)
			{
		int frame_num = boost::lexical_cast<int>(
				((*i).substr((*i).length() - 9, 5)).c_str());

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
	//if(cam_name == "Cam_02")
	//{
	//Skeletonization skeletonizator(frames);
	//for(int i = 90; i < 120; i++)
	//{
	//skeletonizator.generate_skeletonization();
	//skeletonizator.dist_transform_skeletonization(frames[i].depth_img);
	//cv::waitKey(0);
	//}
	//}

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

RGBD_Camera::~RGBD_Camera() {
	// TODO Auto-generated destructor stub
}

void RGBD_Camera::load_timestamps(std::map<int, double>& ts, std::string fn) {
	std::ifstream file;
	file.open(fn.c_str());
	int frame_num(-1);
	double val(-69);
	if (file.is_open()) {
		file >> frame_num;
		file >> val;
		ts[frame_num] = val;
		while (!file.eof()) {
			file >> frame_num;
			file >> val;
			ts[frame_num] = val;
		}

		file.close();
	} else {
		cout << "Error: Failed to open timestamp file " << fn << endl;
	}

}

void RGBD_Camera::load_frame(int frame_num) {
	RGBD_Frame frame;

	frame.frame_num = frame_num - 90;

	//Read images:
	char fn[1024];
	sprintf(fn, "%s/%s/RGB/%05d.png", _dataset_path.c_str(), _cam_name.c_str(),
			frame_num);
	frame.rgb_img = cv::imread(fn, -1);
	if (frame.rgb_img.data == NULL) {
		cout << "Error: failed to read image " << fn << endl;
		exit(-1);
	}

	sprintf(fn, "%s/%s/Depth/%05d.png", _dataset_path.c_str(),
			_cam_name.c_str(), frame_num);
	frame.depth_img = cv::imread(fn, -1);
	if (frame.depth_img.data == NULL) {
		cout << "Error: failed to read image " << fn << endl;
		exit(-1);
	}

	frames[frame_num - 90] = frame;

}

void RGBD_Camera::load_calibration() {
	std::string K_fn(
			_dataset_path + "/" + _cam_name + "/Calibration/K_rgb.txt");
	std::string T_fn(
			_dataset_path + "/" + _cam_name + "/Calibration/T_rgb.txt");

	//Intrinsics:
	//------------------------------------
	std::ifstream K_file;
	K_file.open(K_fn.c_str());
	float K_val[9];
	if (K_file.is_open()) {
		for (int i = 0; i < 9; i++) {
			K_file >> K_val[i];
		}

		K_file.close();
	} else {
		cout << "Error: Failed to open intrinsic calibration file" << endl;
		exit(-1);
	}
	//NB! Cloning because the array with the data will go out of scope when the function returns:
	cv::Mat K = cv::Mat(3, 3, CV_32F, K_val, 3 * sizeof(float)).clone();
	K_rgb = K;
	inv_K_rgb = K.inv();

	for (int i = 0; i < 9; i++) {
		K_d_f3x3.val[i] = K_rgb.at<float>(i);
		inv_K_d_f3x3.val[i] = inv_K_rgb.at<float>(i);
	}

	//DEBUG:
	//cout << "Intrinsics read from file: " << K << endl;

	//------------------------------------

	//Extrinsics:
	//------------------------------------
	std::ifstream T_file;
	T_file.open(T_fn.c_str());
	float T_val[16];
	if (T_file.is_open()) {
		for (int i = 0; i < 16; i++) {
			T_file >> T_val[i];
		}

		T_file.close();
	} else {
		cout << "Error: Failed to open extrinsic calibration file" << endl;
		exit(-1);
	}
	//NB! Cloning because the array with the data will go out of scope when the function returns:
	cv::Mat T = cv::Mat(4, 4, CV_32F, T_val, 4 * sizeof(float)).clone();
	T_rgb = T;

	for (int i = 0; i < 16; i++) {
		T_float4x4.val[i] = T_rgb.at<float>(i);
	}

	//DEBUG:
	//cout << "Extrinsics read from file: " << T << endl;

	//------------------------------------

}

int RGBD_Camera::get_first_frame_num() const {
	return frames.begin()->first;
}

int RGBD_Camera::get_last_frame_num() const {
	return frames.rbegin()->first;
}

int RGBD_Camera::get_total_frame_num() const {
	return frames.size();
}

void RGBD_Camera::create_cam_geom() {

	//Set camera visualization colour:
	//-----------------
	int cam_num = boost::lexical_cast<int>(
			_cam_name.substr(_cam_name.length() - 2, 2).c_str());
	vis_colour = hsv_2_rgb(osg::Vec3(360 * (cam_num / 5.f), 0.6, 0.7));
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
	osg::Matrix half_sz = osg::Matrix::scale(0.7, 0.7, 0.7);
	half_size->setMatrix(half_sz);
	half_size->addChild(cam_axes);
	cam_pose_xform->addChild(half_size);
	//-----------------

	//Camera label:
	//-----------------
	osg::ref_ptr<osg::Group> cam_label = create_3D_label(_cam_name,
			osg::Vec3(0, 0.2, 0), vis_colour);
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

osg::Geode* RGBD_Camera::create_cam_icon(osg::Vec3 vis_colour) {

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(5);
	(*vertices)[0].set(osg::Vec3(0, 0, 0));
	(*vertices)[1].set(osg::Vec3(-0.0561, 0.0421, 0.1));
	(*vertices)[2].set(osg::Vec3(-0.0561, -0.0421, 0.1));
	(*vertices)[3].set(osg::Vec3(0.0561, -0.0421, 0.1));
	(*vertices)[4].set(osg::Vec3(0.0561, 0.0421, 0.1));
	osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(
	GL_LINES, 16);
	(*indices)[0] = 4;
	(*indices)[1] = 1;
	(*indices)[2] = 1;
	(*indices)[3] = 2;
	(*indices)[4] = 2;
	(*indices)[5] = 3;
	(*indices)[6] = 3;
	(*indices)[7] = 4;

	(*indices)[8] = 0;
	(*indices)[9] = 1;
	(*indices)[10] = 0;
	(*indices)[11] = 2;
	(*indices)[12] = 0;
	(*indices)[13] = 3;
	(*indices)[14] = 0;
	(*indices)[15] = 4;

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(vertices.get());
	geom->addPrimitiveSet(indices.get());

	osg::ref_ptr<osg::Geode> cam_vis = new osg::Geode;
	cam_vis->addDrawable(geom.get());

	//Wireframe:
	osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
	pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	cam_vis->getOrCreateStateSet()->setAttribute(pm.get());

	//Turn off lighting:
	cam_vis->getOrCreateStateSet()->setMode( GL_LIGHTING,
			osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	//Set colour:
	osg::ref_ptr<osg::Vec4Array> colours = new osg::Vec4Array;
	colours->push_back(osg::Vec4(vis_colour, 1.0f));
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

cv::Mat RGBD_Camera::get_K_rgb() {
	return K_rgb;
}

float3x3 RGBD_Camera::get_K_f3x3() {
	return K_d_f3x3;
}

cv::Mat RGBD_Camera::get_inv_K_rgb() {
	return inv_K_rgb;
}

float3x3 RGBD_Camera::get_inv_K_f3x3() {
	return inv_K_d_f3x3;
}

cv::Mat RGBD_Camera::get_T_rgb() {
	return T_rgb;
}

float4x4 RGBD_Camera::get_T_f4x4() {
	return T_float4x4;
}

const cv::Mat* RGBD_Camera::get_depth_map(int frame_num) {
	if (frames.find(frame_num) != frames.end()) {
		return &frames[frame_num].depth_img;
	} else {
		//Load the frame from disk:
		load_frame(frame_num);
		if (frames.find(frame_num) != frames.end()) {
			return &frames[frame_num].depth_img;
		} else {
			cout << "Error: frame " << frame_num
					<< " does not exist in the data.";
			exit(-1);
		}
	}
}

cv::Mat* RGBD_Camera::get_rgb_image(int frame_num) {
	if (frames.find(frame_num) != frames.end()) {
		return &frames[frame_num].rgb_img;
	} else {
		//Load the frame from disk:
		load_frame(frame_num);
		if (frames.find(frame_num) != frames.end()) {
			return &frames[frame_num].depth_img;
		} else {
			cout << "Error: frame " << frame_num
					<< " does not exist in the data.";
			exit(-1);
		}
	}
}

std::string RGBD_Camera::get_cam_name() {
	return _cam_name;
}

int RGBD_Camera::get_d_rows() {
	return d_rows;
}

int RGBD_Camera::get_d_cols() {
	return d_cols;
}

osg::Vec3 RGBD_Camera::get_vis_colour() {
	return vis_colour;
}

void RGBD_Camera::segment_frames() {
	//Get a slightly noise reduced and more complete background plate by averaging the first few 10s of frames:
	//--------------------------------
	cv::Mat background_plate_f(d_rows, d_cols, CV_32F, 0.f);
	cv::Mat background_weight(cv::Mat::zeros(d_rows, d_cols, CV_16U));

	for (std::map<int, RGBD_Frame>::iterator i(frames.begin());
			i->first < frames.begin()->first + 60; ++i) {
		//Incremental averaging valid pixels:
		for (int row = 0; row < d_rows; row++) {
			for (int col = 0; col < d_cols; col++) {
				if ((*i).second.depth_img.at<ushort>(row, col) != 0) {
					float w_old = (float) background_weight.at<ushort>(row,
							col);
					background_plate_f.at<float>(row, col) =
							(background_plate_f.at<float>(row, col) * w_old
									+ (*i).second.depth_img.at<ushort>(row, col)
											* 1.0f) / (w_old + 1);
					background_weight.at<ushort>(row, col)++;}
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

	for (std::map<int, RGBD_Frame>::iterator i(frames.begin());
			i != frames.end(); ++i) {
		//Set pixels to zero if they are within the (depth dependent) noise floor of the background plate::
		for (int row = 0; row < d_rows; row++) {
			for (int col = 0; col < d_cols; col++) {

				float3 global_pt = global_coord((*i).first, row, col);

				//Noise floor is depth dependent quantization step times an extra tolerance factor of 4 to get more filtering:
				float noise_floor = 4
						* (2.8e-6 * (background_plate.at<ushort>(row, col))
								* (background_plate.at<ushort>(row, col)));
				if (!((abs(
						(int) background_plate.at<ushort>(row, col)
								- (int) (*i).second.depth_img.at<ushort>(row,
										col)) > noise_floor)
						&& global_pt.x > -2.5 &&//And bounding box segmentation
						global_pt.x < 2.5 &&
						//global_pt.y < 2.5 &&
						//global_pt.y < 2.5 &&
						global_pt.z > 0.6 && global_pt.z < 1.4)) {
					//Point is classifies as background, set it to invalid (zero):
					(*i).second.depth_img.at<ushort>(row, col) = 0;
				}
			}
		}
	}

	//--------------------------------

}

void RGBD_Camera::get_surface_paths(int frame_num,
		std::vector<std::vector<cv::Point> >& surface_paths) {
	//cout << "Computing surface paths" << endl;

	//std::vector< std::vector<cv::Point> > surface_paths;

	//Loop from bottom to top:
	for (int row = d_rows - 1; row >= 0; row--) {

		std::vector<int> mid_cols;
		int col = 0;
		while (col < d_cols) {
			if (frames[frame_num].depth_img.at<ushort>(row, col) != 0) {
				int begin_col = col;
				int thresh = 30;			//30mm - TODO make depth dependent
				if (col < d_cols - 1)
					col++;
				while (abs(
						(int) frames[frame_num].depth_img.at<ushort>(row, col)
								- (int) frames[frame_num].depth_img.at<ushort>(
										row, col - 1)) < thresh) {
					if (col < d_cols - 1)
						col++;
					else
						break;
				}
				int end_col = col;
				int mid_col = (end_col + begin_col) / 2;//(rounding to integer)
				mid_cols.push_back(mid_col);

				//DEBUG: Show midpoint
				//d_img.at<ushort>(row, mid_col) = 5000;

				col++;
			} else {
				col++;
			}
		}

		//Append each detected midpoint from this row to a existing path (if present) otherwise begin a new one:

		for (unsigned int mc = 0; mc < mid_cols.size(); mc++) {
			bool matched_to_existing = false;

			//See if possible to append to existing contour:
			for (unsigned int sp = 0; sp < surface_paths.size(); sp++) {
				cv::Point last_pt(*(surface_paths[sp].end() - 1));
				if (cont_path(frames[frame_num].depth_img, last_pt,
						cv::Point(mid_cols[mc], row)) == true) {
					matched_to_existing = true;
					surface_paths[sp].push_back(cv::Point(mid_cols[mc], row));
				}
			}
			//Or start a new path:
			if (matched_to_existing == false) {
				std::vector<cv::Point> new_path;
				new_path.push_back(cv::Point(mid_cols[mc], row));
				surface_paths.push_back(new_path);
			}
		}

	}

	//Delete paths with fewer than min_length elements because they're probably noise:
	unsigned int min_length = 25;
	std::vector<std::vector<cv::Point> >::iterator sp(surface_paths.begin());

	while (sp != surface_paths.end()) {
		if ((*sp).size() < min_length) {
			surface_paths.erase(sp);
		} else {
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

bool RGBD_Camera::cont_path(cv::Mat& d_img, cv::Point last, cv::Point current) {

	int thresh = 30;	//30mm

	//Does point a come the row below point b?
	if (last.y != current.y + 1) {
		//DEBUG:
		//cout << "Last y: " << last.y << ", Current y: " << current.y << endl;

		return false;
	}

	//Is there a continuous path between them?
	cv::Point left;
	cv::Point right;
	if (last.x > current.x) {
		left = current;
		right = last;
	} else {
		right = current;
		left = last;
	}

	//If points directly above one another:
	if (left.x == right.x) {
		if (d_img.at<ushort>(left.y, left.x) != 0
				&& abs(
						(int) d_img.at<ushort>(right.y, right.x)
								- (int) d_img.at<ushort>(left.y, left.x))
						< thresh) {
			return true;
		} else {
			return false;
		}
	}

	//Check lower and upper horizontal lines for self consistency and consistency with each other:
	for (int i = left.x; i < right.x - 1; i++) {
		if (!(d_img.at<ushort>(left.y, i) != 0
				&& abs(
						(int) d_img.at<ushort>(left.y, i)
								- (int) d_img.at<ushort>(left.y, i + 1))
						< thresh &&		// horizontal consistency
				abs(
						(int) d_img.at<ushort>(right.y, i)
								- (int) d_img.at<ushort>(right.y, i + 1))
						< thresh))		// "
		{
			return false;
		}
	}
	for (int i = left.x; i < right.x; i++) {
		if (!(abs(
				(int) d_img.at<ushort>(left.y, i)
						- (int) d_img.at<ushort>(right.y, i)) < thresh))//vertical consistency
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

void RGBD_Camera::get_surface_paths_3d(int frame_num,
		std::vector<std::vector<float3> >& surface_paths_3d) {

	//Timer t_2d("2d");

	std::vector<std::vector<cv::Point> > surface_paths_2d;
	get_surface_paths(frame_num, surface_paths_2d);

	//t_2d.tock_print();

	//Timer t_3d("3d");

	float3x3 K_d_f3x3;
	for (int i = 0; i < 9; i++)
		K_d_f3x3.val[i] = K_rgb.at<float>(i);

	surface_paths_3d.clear();
	for (unsigned int sp = 0; sp < surface_paths_2d.size(); sp++) {
		std::vector<float3> temp(surface_paths_2d[sp].size());
		surface_paths_3d.push_back(temp);

		for (unsigned int element = 0; element < surface_paths_3d[sp].size();
				element++) {
			int row = surface_paths_2d[sp][element].y;
			int col = surface_paths_2d[sp][element].x;

			//Read depth pixel (converted from mm to m):
			float depth = (((ushort*) (frames[frame_num].depth_img.data))[row
					* frames[frame_num].depth_img.step1() + col]) / 1000.f;
			//Reproject it:
			float3 depth_pix_hom = make_float3(col, row, 1.f);
			float3 vert = depth * ((K_d_f3x3.inverse()) * depth_pix_hom);
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

float3 RGBD_Camera::global_coord(int frame_num, int row, int col) {

	float3x3 K_d_f3x3;
	for (int i = 0; i < 9; i++)
		K_d_f3x3.val[i] = K_rgb.at<float>(i);

	//Reproject it:
	float depth = ((float) frames[frame_num].depth_img.at<ushort>(row, col))
			/ 1000.f;
	float3 depth_pix_hom = make_float3(col, row, 1.f);
	float3 local = depth * ((K_d_f3x3.inverse()) * depth_pix_hom);

	float4 local_hom = make_float4(local, 1.f);

	//Convert cv mat to float4x4
	float4x4 T_rgb_float4x4;
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			T_rgb_float4x4.val[r * 4 + c] = ((float*) T_rgb.data)[r * 4 + c];
		}
	}

	float4 result_hom = T_rgb_float4x4 * local_hom;

	float3 result = make_float3(result_hom);

	return result;
}

void RGBD_Camera::bilateral_filter_frames() {
	cout << "Bilateral filtering frames" << endl;

	for (std::map<int, RGBD_Frame>::iterator i(frames.begin());
			i != frames.end(); ++i) {
		cv::Mat depth_map_float;
		(*i).second.depth_img.convertTo(depth_map_float, CV_32F);
		cv::Mat depth_bilateral_filtered;
		cv::bilateralFilter(depth_map_float, depth_bilateral_filtered, -1, 25,
				2);
		depth_bilateral_filtered.convertTo(((*i).second.depth_img), CV_16U);
	}
}

const osg::Vec3& RGBD_Camera::getVisColour() const {
	return vis_colour;
}

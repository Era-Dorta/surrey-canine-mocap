#include "Skeletonization3D.h"

Skeletonization3D::Skeletonization3D(float merge_treshold_,
		float move_distance_) :
			n_cameras(0), n_frames(0), merge_treshold(merge_treshold_),
			move_distance(move_distance_) {

}

Skeletonization3D::~Skeletonization3D() {
	//dtor
}

void Skeletonization3D::set_cameras(
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr_) {
	camera_arr = camera_arr_;
	skel_arr.clear();
	//Save number of cameras and total number of frames
	n_cameras = camera_arr.size();
	n_frames = camera_arr[0]->get_total_frame_num();

	//Create a Skeleton2D for each camera
	std::vector<boost::shared_ptr<RGBD_Camera> >::iterator i(
			camera_arr.begin());
	for (; i != camera_arr.end(); ++i) {
		boost::shared_ptr<Skeletonization2D> skel(new Skeletonization2D(*i));
		skel_arr.push_back(skel);
	}

	merge_2D_skeletons();
}

const cv::Mat* const Skeletonization3D::get_2D_frame(int cam_num,
		int frame_num) const {
	return skel_arr[cam_num]->get_frame(frame_num);
}

void Skeletonization3D::merge_2D_skeletons() {
	skeleton_frames.reserve(n_frames);
	std::vector<const cv::Mat*> skeletonized_frames;
	skeletonized_frames.resize(n_cameras);

	for (int i = 0; i < n_frames; i++) {
		//for( int i = 0; i < 10; i++){
		//Get all the 2D views of a given frame
		for (unsigned int j = 0; j < skel_arr.size(); j++) {
			skeletonized_frames[j] = skel_arr[j]->get_frame(i);
		}
		//Save the 3D result
		skeleton_frames.push_back(
				merge_2D_skeletons_impl(skeletonized_frames, i));
	}
}

bool Skeletonization3D::get_white_pixel(cv::Mat& img, int &res_row,
		int &res_col, int i_row, int i_col) {
	for (int row = i_row; row < img.rows; row++) {
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

bool Skeletonization3D::get_bottom_white_pixel(cv::Mat& img, int &res_row,
		int &res_col) {
	return get_bottom_white_pixel(img, res_row, res_col, img.rows - 1, 0);
}

bool Skeletonization3D::get_bottom_white_pixel(cv::Mat& img, int &res_row,
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

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::get_simple_3d_projection(
		int cam_num, int frame_num) const {
	//Return vector
	osg::ref_ptr<osg::Vec3Array> skeleton_3d = new osg::Vec3Array();

	const cv::Mat* depth_map;
	const cv::Mat* skeleton_img;
	float3x3 inv_K;

	//Calculate 3D proyections of 2D skeleton images
	//Every image is from a different camera
	depth_map = camera_arr[cam_num]->get_depth_map(frame_num);
	skeleton_img = skel_arr[cam_num]->get_frame(frame_num);
	inv_K = camera_arr[cam_num]->get_inv_K_f3x3();
	int rows = depth_map->rows;
	int cols = depth_map->cols;

	//Generate 3D vertices
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			//If the pixel belongs to the skeleton then calculate it's projection
			if ((int) skeleton_img->at<uchar>(row, col) == 255) {
				//Read depth pixel (converted from mm to m):
				float depth = (((ushort*) (depth_map->data))[row
						* depth_map->step1() + col]) * 0.001f;
				//Depth 0 means background, it should not be in the projection
				//This is just a safe check, depth should never be 0
				if (depth != 0) {
					//Reproject it:
					float3 depth_pix_hom = make_float3(col, row, 1.f);
					float3 vert = depth * (inv_K * depth_pix_hom);
					//Add to array
					skeleton_3d->push_back(osg::Vec3(vert.x, vert.y, vert.z));
				}
			}
		}
	}

	translate_points_to_inside(skeleton_3d.get(), cam_num, move_distance);
	return skeleton_3d.get();
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::get_merged_3d_projection(
		int frame_num) const {
	return skeleton_frames[frame_num];
}

//TODO This is not translating as it should.. dont know what to to
void Skeletonization3D::translate_points_to_inside(
		std::map<osg::Vec2, osg::Vec3>& projection3d, int cam_num,
		float distance) const {
	std::map<osg::Vec2, osg::Vec3>::iterator point;
	float4 aux = make_float4(0, 0, distance, 1);
	float4 res = aux * camera_arr[cam_num]->get_T_f4x4();
	osg::Vec3 translation(res.x, res.y, res.z);

	for (point = projection3d.begin(); point != projection3d.end(); ++point) {
		point->second = point->second + translation;
	}
}

void Skeletonization3D::translate_points_to_inside(
		osg::ref_ptr<osg::Vec3Array> projection3d, int cam_num,
		float distance) const {
	osg::Vec3Array::iterator point;
	osg::Vec3 translation(0, 0, distance);

	for (point = projection3d->begin(); point != projection3d->end(); ++point) {
		*point = *point + translation;
	}
}

void Skeletonization3D::get_global_coord_3d_projection(int cam_num,
		int frame_num, std::map<osg::Vec2, osg::Vec3>& projection3d) const {
	const cv::Mat* depth_map;
	const cv::Mat* skeleton_img;

	//Calculate 3D proyections of 2D skeleton images
	//Every image is from a different camera
	depth_map = camera_arr[cam_num]->get_depth_map(frame_num);
	skeleton_img = skel_arr[cam_num]->get_frame(frame_num);
	float3x3 inv_K = camera_arr[cam_num]->get_inv_K_f3x3();
	float4x4 T = camera_arr[cam_num]->get_T_f4x4();
	int rows = depth_map->rows;
	int cols = depth_map->cols;

	//Generate 3D vertices
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			//If the pixel belongs to the skeleton then calculate it's projection
			if ((int) skeleton_img->at<uchar>(row, col) == 255) {
				//Read depth pixel (converted from mm to m):
				float depth = (((ushort*) (depth_map->data))[row
						* depth_map->step1() + col]) * 0.001f;
				//Depth 0 means background, it should not be in the projection
				//This is just a safe check, depth should never be 0
				if (depth != 0) {
					//Reproject it:
					float3 depth_pix_hom = make_float3(col, row, 1.f);
					float3 vert = depth * (inv_K * depth_pix_hom);
					float4 vert_hom = make_float4(vert, 1.f);
					float4 vert_global = T * vert_hom;
					//Add to array
					projection3d[osg::Vec2(row, col)] = osg::Vec3(vert_global.x,
							vert_global.y, vert_global.z);
				}
			}
		}
	}
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::merge_2D_skeletons_impl(
		std::vector<const cv::Mat*>& skeletonized_frames, int frame_num) {
	//Vector with 3D projections of 2D skeleton from every camera
	std::vector<std::map<osg::Vec2, osg::Vec3> > projection3d_array;

	std::vector<cv::Mat> visited_pixels;

	for (int i = 0; i < n_cameras; i++) {
		//Calculate 3D projection
		std::map<osg::Vec2, osg::Vec3> aux;
		get_global_coord_3d_projection(i, frame_num, aux);

		//Move all points slightly away from its camera, so they represent a
		//point inside the boy and not on the body
		translate_points_to_inside(aux, i, move_distance);
		//TODO A possible optimisation is to multiply camera matrix with this
		//translation, so everything will be done in one operation

		projection3d_array.push_back(aux);

		//Initialise visited pixel matrices
		visited_pixels.push_back(skel_arr[i]->get_frame(frame_num)->clone());
	}

	osg::ref_ptr<osg::Vec3Array> result;

	//result = simple_2D_merge(projection3d_array);

	result = follow_path_2D_merge(visited_pixels, projection3d_array);

	return result.get();
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::simple_2D_merge(
		std::vector<std::map<osg::Vec2, osg::Vec3> >& projection3d_array) {
	//Return vector
	osg::ref_ptr<osg::Vec3Array> result = new osg::Vec3Array();

	//For each projection
	std::vector<std::map<osg::Vec2, osg::Vec3> >::iterator projection3d;
	projection3d = projection3d_array.begin();
	for (; projection3d != projection3d_array.end(); ++projection3d) {
		//For each point
		std::map<osg::Vec2, osg::Vec3>::iterator point;
		for (point = projection3d->begin(); point != projection3d->end();
				++point) {

			osg::Vec3 merged_pixel = point->second;
			int n_pixel_merge = 1;
			cv::Point3f p0;
			p0.x = merged_pixel.x();
			p0.y = merged_pixel.y();
			p0.z = merged_pixel.z();
			float current_dist, smallest_dist = FLT_MAX;

			//For each other projection
			std::vector<std::map<osg::Vec2, osg::Vec3> >::iterator other_projection3d;
			other_projection3d = projection3d + 1;
			for (; other_projection3d != projection3d_array.end();
					++other_projection3d) {

				bool pixel_found = false;

				//For each point in other projection
				std::map<osg::Vec2, osg::Vec3>::iterator other_point,
						to_merge_point;
				other_point = other_projection3d->begin();
				for (; other_point != other_projection3d->end();
						++other_point) {

					cv::Point3f p1;
					p1.x = other_point->second.x();
					p1.y = other_point->second.y();
					p1.z = other_point->second.z();

					current_dist = cv::norm(p0 - p1);
					if (current_dist < smallest_dist) {
						smallest_dist = current_dist;
						if (current_dist < merge_treshold) {
							pixel_found = true;
							to_merge_point = other_point;
						}
					}
				}

				if (pixel_found) {
					n_pixel_merge++;
					merged_pixel = merged_pixel + to_merge_point->second;
					other_projection3d->erase(to_merge_point);
				}

			}

			merged_pixel = merged_pixel / (float) n_pixel_merge;
			result->push_back(merged_pixel);
		}
	}
	return result.get();
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::follow_path_2D_merge(
		std::vector<cv::Mat>& visited_pixels,
		std::vector<std::map<osg::Vec2, osg::Vec3> >& projection3d_array) {
	//Return vector
	osg::ref_ptr<osg::Vec3Array> result = new osg::Vec3Array();

	cv::Point3f p0, p1;

	int skeleton_num_points = 0;

	int n_total_merge = 0;

	//Merge the skeletons, uses the projections to calculate distances and the
	//2D images to follow the bone path
	for (int i = 0; i < n_cameras; i++) {
		//TODO This searches a white pixel beginning on the bottom-left each time
		//if I save the position it fails, also it could be useful
		//to follow the path of the other bones.
		//TODO More ideas to avoid calculating distance, maybe do first box
		//check, if it is "close" on 3 dimensions then calculate the distance.
		int pixel_row = 0, pixel_col = 0;
		osg::Vec3 merged_pixel, aux_pixel;
		int n_pixel_merge;
		bool continue_merge = get_bottom_white_pixel(visited_pixels[i],
				pixel_row, pixel_col);

		while (continue_merge) {
			//Mark found pixel as visited
			visited_pixels[i].at<uchar>(pixel_row, pixel_col) = 0;

			n_pixel_merge = 1;

			merged_pixel =
					projection3d_array[i][osg::Vec2(pixel_row, pixel_col)];

			p0.x = merged_pixel.x();
			p0.y = merged_pixel.y();
			p0.z = merged_pixel.z();

			float current_dist, smallest_dist = FLT_MAX;

			//For each other projection
			std::vector<std::map<osg::Vec2, osg::Vec3> >::iterator other_projection3d;
			other_projection3d = projection3d_array.begin() + i + 1;
			int j = 0;
			for (; other_projection3d != projection3d_array.end();
					++other_projection3d) {

				bool pixel_found = false;

				//For each point in other projection
				std::map<osg::Vec2, osg::Vec3>::iterator other_point,
						to_merge_point;
				other_point = other_projection3d->begin();
				for (; other_point != other_projection3d->end();
						++other_point) {
					cv::Point3f p1;
					p1.x = other_point->second.x();
					p1.y = other_point->second.y();
					p1.z = other_point->second.z();

					current_dist = cv::norm(p0 - p1);
					if (current_dist < smallest_dist) {
						smallest_dist = current_dist;
						if (current_dist < merge_treshold) {
							pixel_found = true;
							to_merge_point = other_point;
						}
					}
				}

				if (pixel_found) {
					n_pixel_merge++;
					n_total_merge++;
					merged_pixel = merged_pixel + to_merge_point->second;
					//Set visited as 0, since it was used
					visited_pixels[j].at<uchar>(to_merge_point->first.x(),
							to_merge_point->first.y()) = 0;
					//Delete it from the 3D projected points container
					other_projection3d->erase(to_merge_point);
				}
				j++;
			}

			skeleton_num_points++;
			merged_pixel = merged_pixel / (float) n_pixel_merge;
			result->push_back(merged_pixel);

			continue_merge = false;
			int next_row, next_col;

			//Try to follow the bone, search for next pixel in its Moore
			//neighbourhood
			if (get_neighbor_white_pixel(visited_pixels[i], pixel_row,
					pixel_col, next_row, next_col)) {
				pixel_row = next_row;
				pixel_col = next_col;
				continue_merge = true;
			} else {
				//If the search fails, find another pixel starting from top-left
				//corner
				continue_merge = get_bottom_white_pixel(visited_pixels[i],
						pixel_row, pixel_col);
			}
		}
	}

	//cout << "skeleton_num_points " << skeleton_num_points << " merged points " << n_total_merge << endl;
	return result.get();
}

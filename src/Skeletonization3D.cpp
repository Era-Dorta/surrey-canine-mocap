#include "Skeletonization3D.h"
#include "DebugUtil.h"

Skeletonization3D::Skeletonization3D(const camVecT& camera_arr_,
		float merge_treshold_, float row_treshold_, float move_distance_) :
			camera_arr(camera_arr_), n_cameras(0), n_frames(0),
			merge_treshold(merge_treshold_), row_treshold(row_treshold_),
			move_distance(move_distance_) {
}

Skeletonization3D::~Skeletonization3D() {
	//dtor
}

void Skeletonization3D::generate_skeletonization() {
	skel_arr.clear();
	//Save number of cameras and total number of frames
	n_cameras = camera_arr.size();
	n_frames = camera_arr[0]->get_total_frame_num();

	//Create a Skeleton2D for each camera
	for (constCamVecIte i = camera_arr.begin(); i != camera_arr.end(); ++i) {
		boost::shared_ptr<Skeletonization2D> skel(new Skeletonization2D(*i));
		skel->generate_skeletonization();
		skel_arr.push_back(skel);
	}
	skel3d_merged_array.resize(n_frames);
	skel2d_cam_array.resize(n_frames * n_cameras);
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::get_simple_3d_projection(
		int cam_num, int frame_num) {
	if (!skel2d_cam_array[frame_num * n_cameras + cam_num].valid()) {
		do_3d_projection(cam_num, frame_num);
	}
	return skel2d_cam_array[frame_num * n_cameras + cam_num];
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::get_merged_3d_projection(
		int frame_num) {
	if (!skel3d_merged_array[frame_num].valid()) {
		merge_2D_skeletons(frame_num);
	}
	return skel3d_merged_array[frame_num];
}

const cv::Mat& Skeletonization3D::get_2D_frame(int cam_num,
		int frame_num) const {
	return skel_arr[cam_num]->get_frame(frame_num);
}

const cv::Mat& Skeletonization3D::get_2D_bin_frame(int cam_num,
		int frame_num) const {
	return skel_arr[cam_num]->get_bin_frame(frame_num);
}

void Skeletonization3D::merge_2D_skeletons(int frame_num) {

	std::vector<const cv::Mat*> skeletonized_frames;

	//Get all the 2D views of a given frame
	for (unsigned int j = 0; j < skel_arr.size(); j++) {
		skeletonized_frames.push_back(&skel_arr[j]->get_frame(frame_num));
	}
	//Save the 3D result
	skel3d_merged_array[frame_num] = merge_2D_skeletons_impl(
			skeletonized_frames, frame_num);
}

void Skeletonization3D::do_3d_projection(int cam_num, int frame_num) {
	//Return vector
	osg::ref_ptr<osg::Vec3Array> skeleton_3d = new osg::Vec3Array();

	//Calculate 3D proyections of 2D skeleton images
	//Every image is from a different camera
	const cv::Mat* depth_map = camera_arr[cam_num]->get_depth_map(frame_num);
	const cv::Mat& skeleton_img = skel_arr[cam_num]->get_frame(frame_num);
	float3x3 inv_K = camera_arr[cam_num]->get_inv_K_f3x3();
	int rows = depth_map->rows;
	int cols = depth_map->cols;

	//Generate 3D vertices
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			//If the pixel belongs to the skeleton then calculate it's projection
			if ((int) skeleton_img.at<uchar>(row, col) == 255) {
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

	translate_points_to_inside(skeleton_3d.get(), cam_num);
	skel2d_cam_array[frame_num * n_cameras + cam_num] = skeleton_3d.get();
}

void Skeletonization3D::translate_points_to_inside(
		osg::ref_ptr<osg::Vec3Array> projection3d, int cam_num) const {
	osg::Vec3Array::iterator point;
	osg::Vec3 translation(0, 0, move_distance);

	for (point = projection3d->begin(); point != projection3d->end(); ++point) {
		*point = *point + translation;
	}
}

void Skeletonization3D::get_global_coord_3d_projection(int cam_num,
		int frame_num, std::map<osg::Vec2, osg::Vec3>& projection3d) const {
	//Calculate 3D proyections of 2D skeleton images
	//Every image is from a different camera
	const cv::Mat* depth_map = camera_arr[cam_num]->get_depth_map(frame_num);
	const cv::Mat& skeleton_img = skel_arr[cam_num]->get_frame(frame_num);
	float3x3 inv_K = camera_arr[cam_num]->get_inv_K_f3x3();
	float4x4 T = camera_arr[cam_num]->get_T_f4x4();
	int rows = depth_map->rows;
	int cols = depth_map->cols;
	//Generate 3D vertices
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			//If the pixel belongs to the skeleton then calculate it's projection
			if ((int) skeleton_img.at<uchar>(row, col) == 255) {
				//Read depth pixel (converted from mm to m):
				float depth = (((ushort*) (depth_map->data))[row
						* depth_map->step1() + col]) * 0.001f;
				//Depth 0 means background, it should not be in the projection
				//This is just a safe check, depth should never be 0
				if (depth != 0) {
					//Reproject it:
					float3 depth_pix_hom = make_float3(col, row, 1.f);
					float3 vert = depth * (inv_K * depth_pix_hom);
					//Move all points slightly away from its camera, so they represent a
					//point inside the boy and not on the body
					vert.z = vert.z + move_distance;
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
		projection3d_array.push_back(aux);
		//Initialise visited pixel matrices
		visited_pixels.push_back(skel_arr[i]->get_frame(frame_num).clone());
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

	//Finds the first white pixel in the bottom-left of one of the 2D skeleton
	//images, then merges it with the closest pixel in the other images.
	for (int i = 0; i < n_cameras; i++) {
		//TODO This searches a white pixel beginning on the bottom-left each time
		//if I save the position it fails, also it could be useful
		//to follow the path of the other bones.
		//TODO More ideas to avoid calculating distance, maybe do first box
		//check, if it is "close" on 3 dimensions then calculate the distance.
		//Third idea, use square distance, since it is faster to calculate
		int pixel_row = 0, pixel_col = 0;
		osg::Vec3 merged_pixel, aux_pixel;
		int n_pixel_merge;
		bool continue_merge = PixelSearch::get_bottom_white_pixel(
				visited_pixels[i], pixel_row, pixel_col);

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

			//For each projection
			std::vector<std::map<osg::Vec2, osg::Vec3> >::iterator other_projection3d;
			other_projection3d = projection3d_array.begin();
			int j = 0;
			for (; other_projection3d != projection3d_array.end();
					++other_projection3d) {

				//If this is current projection, do not average with self
				//pixels, so continue
				if (other_projection3d == projection3d_array.begin() + i) {
					j++;
					continue;
				}

				bool pixel_found = false;

				//For each point in other projection
				std::map<osg::Vec2, osg::Vec3>::iterator other_point,
						to_merge_point;
				other_point = other_projection3d->begin();
				for (; other_point != other_projection3d->end();
						++other_point) {

					//The merging is usually for vertical bones,
					//this helps not to merge vertical with horizontal bones
					//and loose important information. It also increases speed
					//as it avoids calculating many costly distances
					if (p0.z + row_treshold > other_point->second.z()
							&& p0.z - row_treshold < other_point->second.z()) {
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
				}

				if (pixel_found) {
					n_pixel_merge++;
					n_total_merge++;
					merged_pixel = merged_pixel + to_merge_point->second;
					//Set visited as 0, since it was used, but do not deleted
					//from the array of points, so it can be used to average
					//other points
					visited_pixels[j].at<uchar>(to_merge_point->first.x(),
							to_merge_point->first.y()) = 0;
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
			if (PixelSearch::get_neighbor_white_pixel(visited_pixels[i],
					pixel_row, pixel_col, next_row, next_col)) {
				pixel_row = next_row;
				pixel_col = next_col;
				continue_merge = true;
			} else {
				//If the search fails, find another pixel starting from top-left
				//corner
				continue_merge = PixelSearch::get_bottom_white_pixel(
						visited_pixels[i], pixel_row, pixel_col);
			}
		}
	}

	return result.get();
}

int Skeletonization3D::get_n_frames() const {
	return n_frames;
}

int Skeletonization3D::get_d_rows() const {
	return camera_arr.front()->get_d_rows();
}

int Skeletonization3D::get_d_cols() const {
	return camera_arr.front()->get_d_cols();
}

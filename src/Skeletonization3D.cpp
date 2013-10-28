#include "Skeletonization3D.h"

Skeletonization3D::Skeletonization3D(float merge_treshold_)
{
	n_cameras = 0;
	n_frames = 0;
	merge_treshold = merge_treshold_;
}

Skeletonization3D::~Skeletonization3D()
{
	//dtor
}

void Skeletonization3D::set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> > camera_arr_)
{
	camera_arr = camera_arr_;
	skel_arr.clear();
	//Save number of cameras and total number of frames
	n_cameras = camera_arr.size();
	n_frames = camera_arr[0]->get_total_frame_num();

	//Create a Skeleton2D for each camera
	std::vector < boost::shared_ptr<RGBD_Camera> >::iterator i(camera_arr.begin());
	for(; i != camera_arr.end(); ++i){
		boost::shared_ptr<Skeletonization2D> skel(new Skeletonization2D(*i));
		skel_arr.push_back(skel);
	}

	merge_2D_skeletons();
}

const cv::Mat* const Skeletonization3D::get_2D_frame(int cam_num, int frame_num ) const
{
	return skel_arr[cam_num]->get_frame(frame_num);
}

void Skeletonization3D::merge_2D_skeletons()
{
	skeleton_frames.reserve(n_frames);
	std::vector< const cv::Mat* > skeletonized_frames;
	skeletonized_frames.resize(n_cameras);

	for( int i = 0; i < n_frames; i++){
		//Get all the 2D views of a given frame
		for(unsigned int j = 0; j < skel_arr.size(); j++){
			skeletonized_frames[j] = skel_arr[j]->get_frame(i);
		}
		//Save the 3D result
		skeleton_frames.push_back(merge_2D_skeletons_impl(skeletonized_frames, i));
	}
}

bool Skeletonization3D::get_white_pixel( cv::Mat* img, int &res_row, int &res_col, int i_row, int i_col )
{
	for(int row = i_row; row < img->rows; row++)
	{
		for(int col = i_col; col < img->cols; col++)
		{
			//If pixel is white
			if( (int)img->at<uchar>(row, col) == 255){
				res_row = row;
				res_col = col;
				return true;
			}
		}
	}
	return false;
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::get_simple_3d_projection( int cam_num, int frame_num ) const
{
	//Return vector
	osg::ref_ptr< osg::Vec3Array> skeleton_3d = new osg::Vec3Array();

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
	for(int row = 0; row < rows; row++)
	{
		for(int col = 0; col < cols; col++)
		{
			//If the pixel belongs to the skeleton then calculate it's projection
			if( (int)skeleton_img->at<uchar>(row, col) == 255){
				//Read depth pixel (converted from mm to m):
				float depth = (((ushort*)(depth_map->data))[row*depth_map->step1() + col])*0.001f;
				//Depth 0 means background, it should not be in the projection
				//This is just a safe check, depth should never be 0
				if(depth != 0 ){
					//Reproject it:
					float3 depth_pix_hom = make_float3(col, row, 1.f);
					float3 vert = depth*(inv_K*depth_pix_hom);
					//Add to array
					skeleton_3d->push_back(osg::Vec3(vert.x, vert.y, vert.z));
				}
			}
		}
	}

	return skeleton_3d.get();
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::get_merged_3d_projection( int frame_num ) const
{
	return skeleton_frames[frame_num];
}

void Skeletonization3D::get_global_coord_3d_projection(int cam_num, int frame_num, std::map<osg::Vec2, osg::Vec3>& projection3d) const
{
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
	for(int row = 0; row < rows; row++)
	{
		for(int col = 0; col < cols; col++)
		{
			//If the pixel belongs to the skeleton then calculate it's projection
			if( (int)skeleton_img->at<uchar>(row, col) == 255){
				//Read depth pixel (converted from mm to m):
				float depth = (((ushort*)(depth_map->data))[row*depth_map->step1() + col])*0.001f;
				//Depth 0 means background, it should not be in the projection
				//This is just a safe check, depth should never be 0
				if(depth != 0 ){
					//Reproject it:
					float3 depth_pix_hom = make_float3(col, row, 1.f);
					float3 vert = depth*(inv_K*depth_pix_hom);
					float4 vert_hom = make_float4(vert, 1.f);
					float4 vert_global = T*vert_hom;
					//Add to array
					projection3d[osg::Vec2(row, col)] = osg::Vec3(vert_global.x, vert_global.y, vert_global.z);
				}
			}
		}
	}
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::merge_2D_skeletons_impl(
	std::vector<const cv::Mat* >& skeletonized_frames, int frame_num)
{
	//Vector with 3D projections of 2D skeleton from every camera
	std::vector<std::map<osg::Vec2, osg::Vec3> >projection3d_array;
	std::map<osg::Vec2, osg::Vec3> aux;

	std::vector < cv::Mat > visited_pixels;

	for(int i = 0; i < n_cameras; i++){
		//Calculate 3D projection
		get_global_coord_3d_projection(i, frame_num, aux);
		projection3d_array.push_back(aux);

		//Initialise visited pixel matrices
		visited_pixels.push_back(skel_arr[i]->get_frame(frame_num)->clone());
	}


	osg::ref_ptr<osg::Vec3Array> result = simple_2D_merge(&visited_pixels,
			&projection3d_array, &skeletonized_frames);
	return result.get();
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::simple_2D_merge(
		std::vector < cv::Mat >* visited_pixels,
		std::vector<std::map<osg::Vec2, osg::Vec3> >* projection3d_array,
		std::vector<const cv::Mat* >* skeletonized_frames)
{
	//Return vector
	osg::ref_ptr<osg::Vec3Array> result = new osg::Vec3Array();

	cv::Point3f p0, p1;

	int rows = camera_arr[0]->get_d_rows();
	int cols = camera_arr[0]->get_d_cols();
	int pixel_row = 0, pixel_col = 0;
	int skeleton_num_points = 0;
	osg::Vec3 merged_pixel, aux_pixel;
	int total_merge;


	//Merge the skeletons, uses the projections to calculate distances and the
	//2D images to follow the bone path
	for(int i = 0; i < n_cameras; i++){
		while( get_white_pixel(&(*visited_pixels)[i], pixel_row, pixel_col)){
			//Mark found pixel as visited
			(*visited_pixels)[i].at<uchar>(pixel_row, pixel_col) = 0;

			total_merge = 1;

			merged_pixel = (*projection3d_array)[i][osg::Vec2(pixel_row, pixel_col)];

			p0.x = merged_pixel.x();
			p0.y = merged_pixel.y();
			p0.z = merged_pixel.z();

			//We can safely assume that we only have to merge with the images
			//of the next cameras, since we already treated all the pixels
			//in the previous ones
			float smallest_dist [3] = {FLT_MAX, FLT_MAX, FLT_MAX};
			float dist [3] = {FLT_MAX, FLT_MAX, FLT_MAX};
			for(unsigned int j = i + 1; j < skeletonized_frames->size(); j++){
				bool pixel_found = false;
				int aux_row = -1, aux_col = -1;
				for(int row = 0; row < rows; row++)
				{
					for(int col = 0; col < cols; col++)
					{
						if((*skeletonized_frames)[j]->at<uchar>(row, col) == 255){
							aux_pixel = (*projection3d_array)[j][osg::Vec2(row, col)];
							p1.x = aux_pixel.x();
							p1.y = aux_pixel.y();
							p1.z = aux_pixel.z();
							dist[j] = cv::norm(p0 - p1);
							if( dist[j] < smallest_dist[j] ){
								smallest_dist[j] = dist[j];
								if( dist[j] < merge_treshold ){
									aux_row = row;
									aux_col = col;
									pixel_found = true;
								}
							}
						}
					}
				}
				if(pixel_found){
					total_merge++;
					//Set merging pixel as visited
					(*visited_pixels)[j].at<uchar>(aux_row, aux_col) = 0;
					aux_pixel = (*projection3d_array)[j][osg::Vec2(aux_row, aux_col)];
					merged_pixel = merged_pixel + aux_pixel;
				}
			}
			merged_pixel = merged_pixel / (float)total_merge;

			result->push_back(merged_pixel);
			skeleton_num_points++;

			// TODO Instead of calling get_white_pixel each iteration, follow
			//a path
		}
	}
	return result.get();
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::follow_path_2D_merge(
		std::vector < cv::Mat >* visited_pixels,
		std::vector<std::map<osg::Vec2, osg::Vec3> >* projection3d_array,
		std::vector<const cv::Mat* >* skeletonized_frames)
{
	//Return vector
	osg::ref_ptr<osg::Vec3Array> result = new osg::Vec3Array();

	cv::Point3f p0, p1;

	int rows = camera_arr[0]->get_d_rows();
	int cols = camera_arr[0]->get_d_cols();
	int pixel_row = 0, pixel_col = 0;
	int skeleton_num_points = 0;
	osg::Vec3 merged_pixel, aux_pixel;
	int total_merge;


	//Merge the skeletons, uses the projections to calculate distances and the
	//2D images to follow the bone path
	for(int i = 0; i < n_cameras; i++){
		while( get_white_pixel(&(*visited_pixels)[i], pixel_row, pixel_col)){
			//Mark found pixel as visited
			(*visited_pixels)[i].at<uchar>(pixel_row, pixel_col) = 0;

			total_merge = 1;

			merged_pixel = (*projection3d_array)[i][osg::Vec2(pixel_row, pixel_col)];

			p0.x = merged_pixel.x();
			p0.y = merged_pixel.y();
			p0.z = merged_pixel.z();

			//We can safely assume that we only have to merge with the images
			//of the next cameras, since we already treated all the pixels
			//in the previous ones
			float smallest_dist [3] = {FLT_MAX, FLT_MAX, FLT_MAX};
			float dist [3] = {FLT_MAX, FLT_MAX, FLT_MAX};
			for(unsigned int j = i + 1; j < skeletonized_frames->size(); j++){
				bool pixel_found = false;
				int aux_row = -1, aux_col = -1;
				for(int row = 0; row < rows; row++)
				{
					for(int col = 0; col < cols; col++)
					{
						if((*skeletonized_frames)[j]->at<uchar>(row, col) == 255){
							aux_pixel = (*projection3d_array)[j][osg::Vec2(row, col)];
							p1.x = aux_pixel.x();
							p1.y = aux_pixel.y();
							p1.z = aux_pixel.z();
							dist[j] = cv::norm(p0 - p1);
							if( dist[j] < smallest_dist[j] ){
								smallest_dist[j] = dist[j];
								if( dist[j] < merge_treshold ){
									aux_row = row;
									aux_col = col;
									pixel_found = true;
								}
							}
						}
					}
				}
				if(pixel_found){
					total_merge++;
					//Set merging pixel as visited
					(*visited_pixels)[j].at<uchar>(aux_row, aux_col) = 0;
					aux_pixel = (*projection3d_array)[j][osg::Vec2(aux_row, aux_col)];
					merged_pixel = merged_pixel + aux_pixel;
				}
			}
			merged_pixel = merged_pixel / (float)total_merge;

			result->push_back(merged_pixel);
			skeleton_num_points++;

			// TODO Instead of calling get_white_pixel each iteration, follow
			//a path
		}
	}
	return result.get();
}

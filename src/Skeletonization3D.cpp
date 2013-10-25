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
	//TODO Still couses seg fault
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

void Skeletonization3D::get_simple_3d_projection(int cam_num, int frame_num, std::map<osg::Vec2, osg::Vec3>& projection3d) const
{
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
					projection3d[osg::Vec2(row, col)] = osg::Vec3(vert.x, vert.y, vert.z);
				}
			}
		}
	}
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::merge_2D_skeletons_impl(
	std::vector<const cv::Mat* >& skeletonized_frames, int frame_num)
{
	//TODO Skeleton coordinates are relative to each camera transformation
	//they should be in global axis coordinates
	//Return vector
	osg::ref_ptr<osg::Vec3Array> result = new osg::Vec3Array();

	//Vector with 3D projections of 2D skeleton from every camera
	std::vector<std::map<osg::Vec2, osg::Vec3> >projection3d_array;
	std::map<osg::Vec2, osg::Vec3> aux;

	std::vector < cv::Mat > visited_pixels;

	for(int i = 0; i < n_cameras; i++){
		//Calculate 3D projection
		get_simple_3d_projection(i, frame_num, aux);
		projection3d_array.push_back(aux);

		//Initialise visited pixel matrices
		visited_pixels.push_back(skel_arr[i]->get_frame(frame_num)->clone());
	}

	//For each image
		//While still untreated pixels in image
			//Find white pixel
			//Calculate merge
				//Better have area of interest
					//To have area of interest calculate K*d*[x,y,z]'
					//and that gives [u,v,1]
					//If this is calculate for each camera then
					//with tresholds for row and cols we can get
					//areas of interest
				//Calculate distance to other non used pixels
				//If smaller than treshold, merge pixels
			//Set used other image pixels to used
			//Get next white pixel from path

	cv::Point3f p0, p1;

	int rows = camera_arr[0]->get_d_rows();
	int cols = camera_arr[0]->get_d_cols();
	int total_pixels = rows*cols, pixel_row = 0, pixel_col = 0;
	int treated_pixels[3] = {0,0,0};
	int skeleton_num_points = 0;
	osg::Vec3 merged_pixel, aux_pixel;
	int total_merge;

	//Merge the skeletons, uses the projections to calculate distances and the
	//2D images to follow the bone path
	for(int i = 0; i < n_cameras; i++){

		while(treated_pixels[i] < total_pixels){
			if(get_white_pixel(&visited_pixels[i], pixel_row, pixel_col, pixel_row, pixel_col)){
				//Mark found pixel as visited
				visited_pixels[i].at<uchar>(pixel_row, pixel_col) = 0;

				total_merge = 1;

				merged_pixel = (projection3d_array[i])[osg::Vec2(pixel_row, pixel_col)];

				p0.x = merged_pixel.x();
				p0.y = merged_pixel.y();
				p0.z = merged_pixel.z();

				//We can safely asume that we only have to merge with the images
				//of the next cameras, since we already treated all the pixels
				//in the previous ones
				for(unsigned int j = i + 1; j < skeletonized_frames.size(); j++){
					for(int row = 0; row < rows; row++)
					{
						for(int col = 0; col < cols; col++)
						{
							//TODO It should not take more than 1 pixel per image
							//to merge, save prev distance and use new one is distance
							//is smaller
							if(skeletonized_frames[j]->at<uchar>(row, col) == 255){
								aux_pixel = (projection3d_array[j])[osg::Vec2(row, col)];
								p1.x = aux_pixel.x();
								p1.y = aux_pixel.y();
								p1.z = aux_pixel.z();
								if( cv::norm(p0 - p1) < merge_treshold ){
									//Set merging pixel as visited
									visited_pixels[j].at<uchar>(pixel_row, pixel_col) = 0;
									merged_pixel = merged_pixel + aux_pixel;
									total_merge++;
									treated_pixels[j]++;
								}
							}
						}
					}
				}
				merged_pixel = merged_pixel / (float)total_merge;

				result->push_back(merged_pixel);
				skeleton_num_points++;

				treated_pixels[i]++;

				// TODO Instead of calling get_white_pixel each iteration, follow
				//a path
			}else{
				treated_pixels[i] = total_pixels;
			}
		}
	}

	return result.get();
}

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
	//merge_2D_skeletons();
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
		std::vector < boost::shared_ptr<Skeletonization2D> >::iterator j(skel_arr.begin());
		for(; j != skel_arr.end(); ++j){
			skeletonized_frames[n_cameras] = (*j)->get_frame(i);
		}
		//Save the 3D result
		skeleton_frames.push_back(merge_2D_skeletons_impl(skeletonized_frames, i));
	}
}

bool Skeletonization3D::get_white_pixel( cv::Mat* img, int &res_row, int &res_col )
{
	for(int row = 0; row < img->rows; row++)
	{
		for(int col = 0; col < img->cols; col++)
		{
			//If pixel is white
			if( (int)img->at<ushort>(row, col) == 255){
				res_row = row;
				res_col = col;
				return true;
			}
		}
	}
	return false;
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::get_simple_3d_projection( int cam_num, int frame_num )
{
	discarded_pixels = new osg::Vec2Array;
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
			if( (int)skeleton_img->at<ushort>(row, col) == 255){
				//Read depth pixel (converted from mm to m):
				float depth = (((ushort*)(depth_map->data))[row*depth_map->step1() + col])*0.001f;
				//TODO Depth 0 means background, how a background was mark as skeleton???
				//Not a clue, but should not be in the projection
				if(depth != 0 ){
					//Reproject it:
					float3 depth_pix_hom = make_float3(col, row, 1.f);
					float3 vert = depth*(inv_K*depth_pix_hom);
					//Add to array
					skeleton_3d->push_back(osg::Vec3(vert.x, vert.y, vert.z));
				}else{
					discarded_pixels->push_back(osg::Vec2(row, col));
				}
				//cout << "[" << vert.x << "," << vert.y << "," << vert.z  << "]" << endl;
			}
		}
	}
	return skeleton_3d.get();
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::merge_2D_skeletons_impl(
	std::vector<const cv::Mat* >& skeletonized_frames, int frame_num)
{
	//Return vector
	osg::ref_ptr<osg::Vec3Array> result = new osg::Vec3Array();

	//Vector with 3D projections of 2D skeleton from every camera
	std::vector< osg::ref_ptr< osg::Vec3Array> > skel_3D_points_array;
	osg::ref_ptr<osg::Vec3Array> aux;


	for(int i = 0; i < n_cameras; i++){
		//Calculate 3D projection
		aux = get_simple_3d_projection(i, frame_num);
		skel_3D_points_array.push_back(aux.get());

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
	int total_pixels = rows*cols, pixel_row, pixel_col;
	int treated_pixels[3] = {0,0,0};
	int skeleton_num_points = 0;
	osg::Vec3 merged_pixel;
	int total_merge;

	//Merge the skeletons, uses the projections to calculate distances and the
	//2D images to follow the bone path
	for(int i = 0; i < n_cameras; i++){

		while(treated_pixels[i] < total_pixels){
			if(get_white_pixel(&visited_pixels[i], pixel_row, pixel_col)){
				//Mark found pixel as visited
				visited_pixels[i].at<uchar>(pixel_row, pixel_col) = 0;

				total_merge = 1;

				merged_pixel = (*(skel_3D_points_array[i]))[pixel_row*cols + pixel_col];

				p0.x = (*(skel_3D_points_array[i]))[pixel_row*cols + pixel_col].x();
				p0.y = (*(skel_3D_points_array[i]))[pixel_row*cols + pixel_col].y();
				p0.z = (*(skel_3D_points_array[i]))[pixel_row*cols + pixel_col].z();
				//We can safely asume that we only have to merge with the images
				//of the next cameras, since we already treated all the pixels
				//in the previous ones
				for(unsigned int j = i + 1; j < skeletonized_frames.size(); j++){
					for(int row = 0; row < rows; row++)
					{
						for(int col = 0; col < cols; col++)
						{
							p1.x = (*(skel_3D_points_array[j]))[row*cols + col].x();
							p1.y = (*(skel_3D_points_array[j]))[row*cols + col].y();
							p1.z = (*(skel_3D_points_array[j]))[row*cols + col].z();
							if( cv::norm(p0 - p1) < merge_treshold ){
								//Set merging pixel as visited
								visited_pixels[j].at<uchar>(pixel_row, pixel_col) = 0;
								merged_pixel = merged_pixel + (*(skel_3D_points_array[j]))[row*cols + col];
								total_merge++;
								treated_pixels[j]++;
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

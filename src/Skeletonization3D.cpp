#include "Skeletonization3D.h"

Skeletonization3D::Skeletonization3D()
{
	//ctor
}

Skeletonization3D::~Skeletonization3D()
{
	//dtor
}

void Skeletonization3D::set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >* camera_arr_)
{
	camera_arr = camera_arr_;
	skel_arr.clear();
	//Save number of cameras and total number of frames
	n_cameras = camera_arr->size();
	n_frames = (*camera_arr)[0]->get_frames()->size();

	//Create a Skeleton2D for each camera
	std::vector < boost::shared_ptr<RGBD_Camera> >::iterator i(camera_arr->begin());
	for(; i != camera_arr->end(); ++i){
		boost::shared_ptr<Skeletonization2D> skel(new Skeletonization2D((*i)->get_frames()));
		skel_arr.push_back(skel);
	}
	merge_2D_skeletons();
}

void Skeletonization3D::merge_2D_skeletons()
{
	skeleton_frames.reserve(n_frames);
	std::vector< cv::Mat* > skeletonized_frames;
	skeletonized_frames.resize(n_cameras);

	for( int i = 0; i < n_frames; i++){
		std::vector < boost::shared_ptr<Skeletonization2D> >::iterator j(skel_arr.begin());
		for(; j != skel_arr.end(); ++j){
			skeletonized_frames[n_cameras] = (*j)->get_frame(i);
		}
		skeleton_frames.push_back(merge_2D_skeletons_impl(skeletonized_frames, i));
	}
}

osg::ref_ptr<osg::Vec3Array> Skeletonization3D::merge_2D_skeletons_impl(
	std::vector< cv::Mat* >& skeletonized_frames, int frame_num)
{
	//Merge method
	//Transform the images to 3D world
	//Travel thrugh pixels -> Have a matrix of travelled pixels?
	// Calculate new vector of mean points
	int rows = (*camera_arr)[0]->get_d_rows();
	int cols = (*camera_arr)[0]->get_d_cols();
	//Return vector
	osg::ref_ptr<osg::Vec3Array> result = new osg::Vec3Array(rows*cols);

	std::vector< osg::ref_ptr< osg::Vec3Array> > frames_3d;
	osg::ref_ptr<osg::Vec3Array> aux;
	for(int i = 0; i < n_cameras; i++){
		aux = new osg::Vec3Array(rows*cols);
		frames_3d.push_back(aux);
	}

	cv::Mat* depth_map;
	float3x3 inv_K;

	//Calculate 3D proyections of 2D skeleton images
	//Every image is from a different camera
	for(int i = 0; i < skeletonized_frames.size(); i++){
		depth_map = (*camera_arr)[i]->get_depth_map(frame_num);
		inv_K = (*camera_arr)[i]->get_inv_K_f3x3();
		//Generate 3D vertices
		for(int row = 0; row < rows; row++)
		{
			for(int col = 0; col < cols; col++)
			{
				//Read depth pixel (converted from mm to m):
				float depth = (((ushort*)(depth_map->data))[row*depth_map->step1() + col])*0.001f;
				//Reproject it:
				float3 depth_pix_hom = make_float3(col, row, 1.f);
				float3 vert = depth*(inv_K*depth_pix_hom);
				//Add to array (negate y and z to correct orientation)://11/07/2013 Update - don't negate
				(*(frames_3d[i]))[row*cols + col].set(vert.x, vert.y, vert.z);
			}
		}
	}

	//Go through each pixel of each image, check if other has pixels
	//inside treshold, if the do, erase pixel in others and actual and
	//put new pixel in the mean
	// If no near treshold, delete current add it to res image as is


}

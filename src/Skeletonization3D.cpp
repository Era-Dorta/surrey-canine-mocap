#include "Skeletonization3D.h"

Skeletonization3D::Skeletonization3D()
{
	//ctor
}

Skeletonization3D::~Skeletonization3D()
{
	//dtor
}

void Skeletonization3D::set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr)
{
	skel_arr.clear();
	for(int i = 0; i < camera_arr.size(); i++){
		boost::shared_ptr<Skeletonization2D> skel(new Skeletonization2D(camera_arr[i]->get_frames()));
		skel_arr.push_back(skel);
	}
}

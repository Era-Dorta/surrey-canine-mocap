#ifndef SKELETONIZATION3D_H
#define SKELETONIZATION3D_H

#include "Skeletonization2D.h"
#include "RGBDCamera.h"

#include <vector>

#include "boost/shared_ptr.hpp"

class Skeletonization3D
{
	public:
		Skeletonization3D();
		virtual ~Skeletonization3D();
		void set_cameras(std::vector < boost::shared_ptr<RGBD_Camera> >& camera_arr);
	protected:
	private:
		//Vector of Skeletonization class, there is one instance
		//for each camera
		std::vector < boost::shared_ptr<Skeletonization2D> > skel_arr;
};

#endif // SKELETONIZATION3D_H

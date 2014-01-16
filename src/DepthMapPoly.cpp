/*
 * DepthMapPoly.cpp
 *
 *  Created on: 11 Jul 2013
 *      Author: cm00215
 */

#include "DepthMapPoly.h"

using std::cout;
using std::endl;

DepthMapPoly::DepthMapPoly() :
			depth_poly_geode(new osg::Geode) {

}

void DepthMapPoly::init(int _rows, int _cols, osg::Vec3 _vis_colour) {
	rows = _rows;
	cols = _cols;
	vis_colour = osg::Vec4(_vis_colour, 1.f);
	depth_poly_geode = create_depth_map_geode();
}

DepthMapPoly::~DepthMapPoly() {
	// TODO Auto-generated destructor stub
}

osg::Geode* DepthMapPoly::create_depth_map_geode() {
	osg::ref_ptr<osg::Geode> dm_geode = new osg::Geode;

	//Allocate vertices and face indices:
	//------------------------
	//Total vertices (including invalid zero depth ones) is rows*cols:
	dm_vertices = new osg::Vec3Array(rows * cols);
	dm_normals = new osg::Vec3Array(rows * cols);
	//Maximum number of faces generates is  2*(rows - 1)*(cols - 1) (x3 to get number of vertex indices):
	int max_faces = 2 * (rows - 1) * (cols - 1);
	dm_vert_indices = new osg::DrawElementsUInt(GL_TRIANGLES, 3 * max_faces);
	//------------------------

	//Make geometry node:
	//------------------------
	dm_geometry = new osg::Geometry;
	dm_geometry->setDataVariance(osg::Object::DYNAMIC);
	dm_geometry->setVertexArray(dm_vertices);
	dm_geometry->setNormalArray(dm_normals);
	dm_geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	dm_geometry->addPrimitiveSet(dm_vert_indices);
	//Use VBO to improve performance:
	dm_geometry->setUseVertexBufferObjects(true);
	dm_geometry->setUseDisplayList(false);

	//Polygonize depth map of current frame:
	//polygonise_depth_map(depth_map, K, discon_thresh,
	//		dm_vertices,
	//		dm_normals,
	//		dm_vert_indices);

	dm_geode->addDrawable(dm_geometry);
	//------------------------

	//Set all the vertex colours to a single value:
	//(note: still making an entire array, so that I have the option of populating it with RGB image colours)
	//------------------------
	dm_colours = new osg::Vec4Array(rows * cols);
	for (int i = 0; i < rows * cols; i++) {
		(*dm_colours)[i] = vis_colour;
	}
	dm_geometry->setColorArray(dm_colours.get());
	dm_geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	//------------------------

	return dm_geode.release();
}

void DepthMapPoly::polygonise_depth_map(cv::Mat* depth_map, cv::Mat* rgb_image,
		cv::Mat K, osg::Vec3 vis_colour, bool with_colour, float alpha) {

	//Timer t_poly_dm("Polygonize depth map cuda vec");
	//t_poly_dm.tick();

//Slows it down too much:
//	//DEBUG TEST: Bilateral filter depth maps before pologonizing:
//	//-------------------------------------
//
//	cv::Mat depth_map_float;
//	depth_map->convertTo(depth_map_float, CV_32F);
//	cv::Mat depth_bilateral_filtered;
//	cv::bilateralFilter(depth_map_float, depth_bilateral_filtered,-1, 20, 1);
//	depth_bilateral_filtered.convertTo((*depth_map), CV_16U);
//
//	//-------------------------------------

	float3x3 K_d_f3x3;
	for (int i = 0; i < 9; i++)
		K_d_f3x3.val[i] = K.at<float>(i);

	//Work on one 2x2 block of pixels at a time, generating between 0 and 2 triangles
	//depending on the depth discontinuity test
	//(generates the vertices in the camera's local coordinate system)

	int rows = depth_map->rows;
	int cols = depth_map->cols;

	//Timer tv("Gen vertices cuda vec");
	//tv.tick();

	//Generate 3D vertices:
	//----------------------------------------
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			//Read depth pixel (converted from mm to m):
			float depth = (((ushort*) (depth_map->data))[row
					* depth_map->step1() + col]) / 1000.f;
			//Reproject it:
			float3 depth_pix_hom = make_float3(col, row, 1.f);
			float3 vert = depth * ((K_d_f3x3.inverse()) * depth_pix_hom);
			//Add to array (negate y and z to correct orientation)://11/07/2013 Update - don't negate
			(*dm_vertices)[row * cols + col].set(vert.x, vert.y, vert.z);
		}
	}
	//----------------------------------------

	//tv.tock_print();

	//Generate normals:
	//----------------------------------------
	float3 vert00;
	float3 vert01;
	float3 vert10;
	float3 unnorm_norm;
	float3 norm;

	//Timer tn("Gen normals cuda vec");
	//tn.tick();

	//(Note: doesn't do the last row and column)
	for (int row = 0; row < rows - 1; row++) {
		for (int col = 0; col < cols - 1; col++) {
			//Compute cross product:
			vert00 = MiscUtils::osgvec_2_float3(
					(*dm_vertices)[row * cols + col]);
			vert01 = MiscUtils::osgvec_2_float3(
					(*dm_vertices)[row * cols + (col + 1)]);
			vert10 = MiscUtils::osgvec_2_float3(
					(*dm_vertices)[(row + 1) * cols + col]);
			unnorm_norm = cross((vert10 - vert00), (vert01 - vert00));
			//Normalize:
			float mag = length(unnorm_norm);
			if (mag > 0) {
				norm = unnorm_norm / mag;
			} else {
				norm = make_float3(0.f, 0.f, 0.f);
			}
			//Set normal array value:
			(*dm_normals)[row * cols + col].x() = norm.x;
			(*dm_normals)[row * cols + col].y() = norm.y;
			(*dm_normals)[row * cols + col].z() = norm.z;
		}
	}
	//----------------------------------------

	//tn.tock_print();

	//Generate colours:
	//----------------------------------------
//	//DEBUG:
//	std::string ws = (with_colour == true) ? "true" : "false";
//	cout << "with_colour: " << ws << endl;

	if (with_colour == true) {
		//float alpha = 0.3;
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				(*dm_colours)[row * cols + col] = osg::Vec4(
						alpha * vis_colour[0]
								+ (1 - alpha)
										* rgb_image->at<cv::Vec3b>(row, col)[2]
										/ 255.f,
						alpha * vis_colour[1]
								+ (1 - alpha)
										* rgb_image->at<cv::Vec3b>(row, col)[1]
										/ 255.f,
						alpha * vis_colour[2]
								+ (1 - alpha)
										* rgb_image->at<cv::Vec3b>(row, col)[0]
										/ 255.f, 1.f);
			}
		}
		//Turn off lighting:
		dm_geometry->getOrCreateStateSet()->setMode( GL_LIGHTING,
				osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	} else {
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				(*dm_colours)[row * cols + col] = osg::Vec4(vis_colour, 1.f);
			}
		}
		//Turn on lighting:
		dm_geometry->getOrCreateStateSet()->setMode( GL_LIGHTING,
				osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	}
	//----------------------------------------

	//Timer tf("Gen faces cuda vec");
	//tf.tick();

	//Generate faces:
	//----------------------------------------
	int current_face_index = 0;
	for (int row = 0; row < (rows - 1); row++) {
		for (int col = 0; col < (cols - 1); col++) {

			//Face and vertex indices for upper left triangle:
			current_face_index = row * (cols - 1) + col;
			int v_1 = row * cols + col;
			int v_2 = (row + 1) * cols + col;
			int v_3 = row * cols + (col + 1);

			//Adaptive threshold for depth discontinuities of according to distance (bearing in mind noise/quantization //(Khoshelham2012)):
			float z = (*dm_vertices)[v_1].z();
			float discon_thresh = 0.04f + 2.2 * 0.0028 * z * z;

			//Check validity of upper left triangle
			//(note that z-values must be negative as +z is now out of screen)://11/07/2013 Update - don't negate
			if ((*dm_vertices)[v_1].z() > 0 && (*dm_vertices)[v_2].z() > 0
					&& (*dm_vertices)[v_3].z() > 0
					&& (fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_2].z())
							< discon_thresh)
					&& (fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_3].z())
							< discon_thresh)) {
				//Triangle is valid, add it:
				(*dm_vert_indices)[current_face_index * 3 + 0] = v_1;
				(*dm_vert_indices)[current_face_index * 3 + 1] = v_2;
				(*dm_vert_indices)[current_face_index * 3 + 2] = v_3;
			} else {
				//Triangle is not valid, add 0's:
				(*dm_vert_indices)[current_face_index * 3 + 0] = 0;
				(*dm_vert_indices)[current_face_index * 3 + 1] = 0;
				(*dm_vert_indices)[current_face_index * 3 + 2] = 0;
			}

			//Face and vertex indices for lower right triangle:
			current_face_index = ((rows - 1) * (cols - 1) - 1)
					+ row * (cols - 1) + col;
			v_1 = (row + 1) * cols + (col + 1);
			v_2 = row * cols + (col + 1);
			v_3 = (row + 1) * cols + col;

			//Check validity of lower right triangle:
			//(note that z-values must be negative as +z is now out of screen)//11/07/2013 - Update don't negate
			if ((*dm_vertices)[v_1].z() > 0 && (*dm_vertices)[v_2].z() > 0
					&& (*dm_vertices)[v_3].z() > 0
					&& (fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_2].z())
							< discon_thresh)
					&& (fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_3].z())
							< discon_thresh)) {
				//Triangle is valid, add it:
				(*dm_vert_indices)[current_face_index * 3 + 0] = v_1;
				(*dm_vert_indices)[current_face_index * 3 + 1] = v_2;
				(*dm_vert_indices)[current_face_index * 3 + 2] = v_3;
			} else {
				//Triangle is not valid, add 0's:
				(*dm_vert_indices)[current_face_index * 3 + 0] = 0;
				(*dm_vert_indices)[current_face_index * 3 + 1] = 0;
				(*dm_vert_indices)[current_face_index * 3 + 2] = 0;
			}

		}
	}
	//----------------------------------------

	//tf.tock_print();

	//t_poly_dm.tock_print();

	//That's dirty:
	dm_vertices->dirty();
	dm_normals->dirty();
	dm_colours->dirty();
	dm_vert_indices->dirty();
	dm_geometry->dirtyBound();

}

//void DepthMapPoly::polygonise_depth_map_cv(cv::Mat* depth_map, cv::Mat K, float discon_thresh)
//{
//
//	//NB! This seems to be much slower than my original CUDA vec version, therefore dont use it.
//
//	Timer t_poly_dm("Polygonize depth map OpenCV vec");
//	t_poly_dm.tick();
//
//	//Work on one 2x2 block of pixels at a time, generating between 0 and 2 triangles
//	//depending on the depth discontinuity test
//	//(generates the vertices in the camera's local coordinate system)
//
//	if(depth_map->data == NULL)
//	{
//		cout << "Error: attempted to access invalid image data" << endl;
//		exit(-1);
//	}
//
//	//Timer tv("Gen vertices");
//	//tv.tick();
//
//	cv::Mat K_inv_1 = K.inv().t();
//	cv::Mat K_inv(K_inv_1.clone());
//
//
//	//Generate 3D vertices:
//	//----------------------------------------
//	for(int row = 0; row < rows; row++)
//	{
//		for(int col = 0; col < cols; col++)
//		{
//			//Read depth pixel (converted from mm to m):
//			float depth = (((ushort*)(depth_map->data))[row*depth_map->step1() + col])/1000.f;
//			//Reproject it:
//			cv::Point3f depth_pix_hom(col, row, 1.f);
//			cv::Mat vert_mat = depth*(K_inv*cv::Mat(depth_pix_hom));
//			//cv::Point3f vert(vert_mat);
//			//Add to array (negate y and z to correct orientation):
//			//(*dm_vertices)[row*cols + col].set(vert.x, -vert.y, -vert.z);
//			(*dm_vertices)[row*cols + col].set(vert_mat.at<float>(0), vert_mat.at<float>(1), vert_mat.at<float>(2));
//		}
//	}
//	//----------------------------------------
//
//	//tv.tock_print();
//
//	//Timer tn("Gen normals");
//	//tn.tick();
//
//	//Generate normals:
//	//----------------------------------------
//	osg::Vec3f vert00;
//	osg::Vec3f vert01;
//	osg::Vec3f vert10;
//	osg::Vec3f unnorm_norm;
//	osg::Vec3f norm;
//
//	//(Note: doesn't do the last row and column)
//	for(int row = 0; row < rows-1; row++)
//	{
//		for(int col = 0; col < cols-1; col++)
//		{
//			//Compute cross product:
//			vert00 =  (*dm_vertices)[row*cols + col];
//			vert01 =  (*dm_vertices)[row*cols + (col+1)];
//			vert10 =  (*dm_vertices)[(row+1)*cols + col];
//			unnorm_norm = (vert10-vert00)^(vert01-vert00);//^ is the osg cross product operator
////			//Normalize:
////			float mag = unnorm_norm.length();
////			if(mag > 0)
////			{
////				norm = unnorm_norm/mag;
////			}
////			else
////			{
////				norm = osg::Vec3f(0.f,0.f,0.f);
////			}
//			unnorm_norm.normalize();
//			norm = unnorm_norm;
//			//Set normal array value:
//			(*dm_normals)[row*cols + col].x() = norm.x();
//			(*dm_normals)[row*cols + col].y() = norm.y();
//			(*dm_normals)[row*cols + col].z() = norm.z();
//		}
//	}
//	//----------------------------------------
//
//	//tn.tock_print();
//
//	//Timer tf("Gen faces");
//	//tf.tick();
//
//	//Generate faces:
//	//----------------------------------------
//	int current_face_index = 0;
//	for(int row = 0; row < (rows-1); row++)
//	{
//		for(int col = 0; col < (cols-1); col++)
//		{
//
//			//Face and vertex indices for upper left triangle:
//			current_face_index = row*(cols-1) + col;
//			int v_1 = row*cols + col;
//			int v_2 = (row+1)*cols + col;
//			int v_3 = row*cols + (col+1);
//
//			//Check validity of upper left triangle
//			//(note that z-values must be negative as +z is now out of screen)://11/07/2013 - using positive z's
//			if((*dm_vertices)[v_1].z() > 0 &&
//					(*dm_vertices)[v_2].z() > 0 &&
//					(*dm_vertices)[v_3].z() > 0 &&
//					(fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_2].z()) < discon_thresh) &&
//					(fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_3].z()) < discon_thresh)
//			)
//			{
//				//Triangle is valid, add it:
//				(*dm_vert_indices)[current_face_index*3 + 0] = v_1;
//				(*dm_vert_indices)[current_face_index*3 + 1] = v_2;
//				(*dm_vert_indices)[current_face_index*3 + 2] = v_3;
//			}
//			else
//			{
//				//Triangle is not valid, add 0's:
//				(*dm_vert_indices)[current_face_index*3 + 0] = 0;
//				(*dm_vert_indices)[current_face_index*3 + 1] = 0;
//				(*dm_vert_indices)[current_face_index*3 + 2] = 0;
//			}
//
//
//			//Face and vertex indices for lower right triangle:
//			current_face_index = ((rows - 1)*(cols - 1) - 1) + row*(cols - 1) + col;
//			v_1 = (row+1)*cols + (col+1);
//			v_2 = row*cols + (col+1);
//			v_3 = (row+1)*cols + col;
//
//			//Check validity of lower right triangle:
//			//(note that z-values must be negative as +z is now out of screen)//11/07/2013 - using positive z's
//			if((*dm_vertices)[v_1].z() > 0 &&
//					(*dm_vertices)[v_2].z() > 0 &&
//					(*dm_vertices)[v_3].z() > 0 &&
//					(fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_2].z()) < discon_thresh) &&
//					(fabs((*dm_vertices)[v_1].z() - (*dm_vertices)[v_3].z()) < discon_thresh)
//			)
//			{
//				//Triangle is valid, add it:
//				(*dm_vert_indices)[current_face_index*3 + 0] = v_1;
//				(*dm_vert_indices)[current_face_index*3 + 1] = v_2;
//				(*dm_vert_indices)[current_face_index*3 + 2] = v_3;
//			}
//			else
//			{
//				//Triangle is not valid, add 0's:
//				(*dm_vert_indices)[current_face_index*3 + 0] = 0;
//				(*dm_vert_indices)[current_face_index*3 + 1] = 0;
//				(*dm_vert_indices)[current_face_index*3 + 2] = 0;
//			}
//
//		}
//	}
//	//----------------------------------------
//
//	//tf.tock_print();
//
//	t_poly_dm.tock_print();
//}


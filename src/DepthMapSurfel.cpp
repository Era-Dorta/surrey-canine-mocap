
#include "DepthMapSurfel.h"

using std::cout;
using std::endl;

DepthMapSurfel::DepthMapSurfel(): depth_surfel_geode(new osg::Geode)
{
	rows = 0;
	cols = 0;
}

void DepthMapSurfel::init(int _rows, int _cols, osg::Vec3 _vis_colour)
{
	rows = _rows;
	cols = _cols;
	vis_colour = osg::Vec4(_vis_colour, 1.f);
	depth_surfel_geode = create_depth_map_surfel_geode();
}

DepthMapSurfel::~DepthMapSurfel()
{
}

osg::Geode* DepthMapSurfel::create_depth_map_surfel_geode(void)
{
	osg::ref_ptr<osg::Geode> dm_geode = new osg::Geode;

	//Allocate vertices and face indices:
	//------------------------
	//Total vertices (including invalid zero depth ones) is rows*cols:
	dm_vertices = new osg::Vec3Array(rows*cols);
	dm_normals = new osg::Vec3Array(rows*cols);
	dm_attributes = new osg::Vec3Array(rows*cols);
	//------------------------

	//Make geometry node:
	//------------------------
	dm_geometry = new osg::Geometry;
	dm_geometry->setDataVariance(osg::Object::DYNAMIC);
	dm_geometry->setVertexArray(dm_vertices);
	dm_geometry->setNormalArray(dm_normals);
	dm_geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

	//Note: use index not assigned to a built in attribute
	//(see http://www.opengl.org/sdk/docs/tutorials/ClockworkCoders/attributes.php)
	dm_geometry->setVertexAttribArray(6, dm_attributes.get());
	dm_geometry->setVertexAttribBinding(6, osg::Geometry::BIND_PER_VERTEX);
	dm_geometry->addPrimitiveSet(
				new osg::DrawArrays(GL_POINTS, 0, dm_vertices->size()) );
	//Use VBO to improve performance:
	dm_geometry->setUseVertexBufferObjects(true);
	dm_geometry->setUseDisplayList(false);

	dm_geode->addDrawable(dm_geometry);
	//------------------------

	//Set all the vertex colours to a single value:
	//(note: still making an entire array, so that I have the option of populating it with RGB image colours)
	//------------------------
	dm_colours = new osg::Vec4Array(rows*cols);
	for(int i = 0; i < rows*cols; i++)
	{
		(*dm_colours)[i] = vis_colour;
	}
	dm_geometry->setColorArray(dm_colours.get());
	dm_geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	//------------------------

	//Enable splat rendering:
	//------------------------
	enable_splat_rendering(dm_geode);
	//------------------------

	return dm_geode.release();
}

void DepthMapSurfel::surfelise_depth_map(
		const cv::Mat* depth_map,
		const cv::Mat* rgb_image,
		const cv::Mat K,
		osg::Vec3 vis_colour,
		bool with_colour,
		float alpha)
{

	//Timer t_poly_dm("Polygonize depth map cuda vec");
	//t_poly_dm.tick();

	float3x3 K_d_f3x3;
	for(int i = 0; i<9; i++)
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
	for(int row = 0; row < rows; row++)
	{
		for(int col = 0; col < cols; col++)
		{
			//Read depth pixel (converted from mm to m):
			float depth = (((ushort*)(depth_map->data))[row*depth_map->step1() + col])/1000.f;
			//Reproject it:
			float3 depth_pix_hom = make_float3(col, row, 1.f);
			float3 vert = depth*((K_d_f3x3.inverse())*depth_pix_hom);
			//Add to array (negate y and z to correct orientation)://11/07/2013 Update - don't negate
			(*dm_vertices)[row*cols + col].set(vert.x, vert.y, vert.z);
		}
	}
	//----------------------------------------

	//tv.tock_print();

	//Generate normals:
	//----------------------------------------

	//Timer tn("Gen normals cuda vec");
	//tn.tick();

	float3 unnorm_norm;
	float3 norm;

	float3 vert_0;
	float3 vert_r;
	float3 vert_b;
	float3 vert_l;
	float3 vert_t;

	float3 vec_hori;
	float3 vec_vert;

	float depth_thresh = 20e-3;//20mm//TODO - depth dependent? or better still, use an angle threshold

	//(Note: doesn't do the first and last row and column)
	for(int row = 1; row < rows-1; row++)
	{
		for(int col = 1; col < cols-1; col++)
		{

			//Vertices of central vertex and its 4-neighbours
			vert_0 =  osgvec_2_float3((*dm_vertices)[row*cols + col]);
			vert_r =  osgvec_2_float3((*dm_vertices)[row*cols + (col+1)]);
			vert_b =  osgvec_2_float3((*dm_vertices)[(row+1)*cols + col]);
			vert_l =  osgvec_2_float3((*dm_vertices)[row*cols + (col-1)]);
			vert_t =  osgvec_2_float3((*dm_vertices)[(row-1)*cols + col]);

			//(Try central difference, if not possible, then fall back on single sided, and failing that use axis vector)

			//Get best 'horizontal' vector:
			if(fabs(vert_0.z - vert_l.z) < depth_thresh &&
					fabs(vert_0.z - vert_r.z) < depth_thresh)
			{
				vec_hori = vert_r - vert_l;
			}
			else if(fabs(vert_0.z - vert_r.z) < depth_thresh)
			{
				vec_hori = vert_r - vert_0;
			}
			else if(fabs(vert_0.z - vert_l.z) < depth_thresh)
			{
				vec_hori = vert_0 - vert_l;
			}
			else
			{
				vec_hori = make_float3(1,0,0);
			}

			//Get best 'vertical' vector:
			if(fabs(vert_0.z - vert_t.z) < depth_thresh &&
					fabs(vert_0.z - vert_b.z) < depth_thresh)
			{
				vec_vert = vert_b - vert_t;
			}
			else if(fabs(vert_0.z - vert_b.z) < depth_thresh)
			{
				vec_vert = vert_b - vert_0;
			}
			else if(fabs(vert_0.z - vert_t.z) < depth_thresh)
			{
				vec_vert = vert_0 - vert_t;
			}
			else
			{
				vec_vert = make_float3(0,1,0);
			}

			//Cross product:
			unnorm_norm = cross(vec_vert, vec_hori);

			//Normalize:
			float mag = length(unnorm_norm);
			if(mag > 0)
			{
				norm = unnorm_norm/mag;
			}
			else
			{
				norm = make_float3(0.f,0.f,1.f);
			}

			//Set normal array value:
			(*dm_normals)[row*cols + col].x() = norm.x;
			(*dm_normals)[row*cols + col].y() = norm.y;
			(*dm_normals)[row*cols + col].z() = norm.z;
		}
	}
	//----------------------------------------

	//tn.tock_print();

	//Generate colours:
	//----------------------------------------

	if(with_colour == true)
	{
		//float alpha = 0.3;
		for(int row = 0; row < rows; row++)
		{
			for(int col = 0; col < cols; col++)
			{
				(*dm_colours)[row*cols + col] = osg::Vec4(
						alpha*vis_colour[0] + (1-alpha)*rgb_image->at<cv::Vec3b>(row,col)[2]/255.f,
						alpha*vis_colour[1] + (1-alpha)*rgb_image->at<cv::Vec3b>(row,col)[1]/255.f,
						alpha*vis_colour[2] + (1-alpha)*rgb_image->at<cv::Vec3b>(row,col)[0]/255.f,
						1.f);
			}
		}
		//Turn off lighting:
		dm_geometry->getOrCreateStateSet()->setMode( GL_LIGHTING,
			osg::StateAttribute::OFF |osg::StateAttribute::OVERRIDE);
	}
	else
	{
		for(int row = 0; row < rows; row++)
		{
			for(int col = 0; col < cols; col++)
			{
				(*dm_colours)[row*cols + col] = osg::Vec4(
						vis_colour,
						1.f);
			}
		}
		//Turn on lighting:
		dm_geometry->getOrCreateStateSet()->setMode( GL_LIGHTING,
				osg::StateAttribute::ON |osg::StateAttribute::OVERRIDE);
	}
	//----------------------------------------

	//Generate point radii:
	//----------------------------------------
	for(int row = 0; row < rows; row++)
	{
		for(int col = 0; col < cols; col++)
		{

			float good_measure = 1.2f;//(to avoid small gaps between surfels which caused them to go missing when re-rendered from the same POV)
			float r = 0.01;
			if(fabs((*dm_normals)[row*cols + col].z()) > 0.2588)//(75 degree angle, Keller2013)
			{
				//Sample spacing-based radius: (From Weise, ICCV 2009):
				r = good_measure*fabs((1.0/sqrt(2.0))*(((*dm_vertices)[row*cols + col].z())/K_d_f3x3.val[0])/
					((*dm_normals)[row*cols + col].z()));
			}
			else
			{
				r = good_measure*fabs((1.0/sqrt(2.0))*(((*dm_vertices)[row*cols + col].z())/K_d_f3x3.val[0])/
						(0.2588));
			}

			(*dm_attributes)[row*cols + col] = osg::Vec3(r,0,0);

		}
	}
	//----------------------------------------

	//t_poly_dm.tock_print();

	//That's dirty:
	dm_vertices->dirty();
	dm_normals->dirty();
	dm_colours->dirty();
	dm_attributes->dirty();
	dm_geometry->dirtyBound();

}

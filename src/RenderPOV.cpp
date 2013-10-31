#include "RenderPOV.h"

RenderPOV::RenderPOV(int tgt_width, int tgt_height) {

	//Create graphics context:
	//----------
	osg::ref_ptr<osg::GraphicsContext::Traits> traits =
			new osg::GraphicsContext::Traits;
	traits->x = 0;
	traits->y = 0;
	traits->width = tgt_width;
	traits->height = tgt_height;
	traits->red = 8;
	traits->green = 8;
	traits->blue = 8;
	traits->alpha = 8;
	traits->windowDecoration = false;
	traits->pbuffer = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;

	osg::ref_ptr<osg::GraphicsContext> pbuffer =
			osg::GraphicsContext::createGraphicsContext(traits.get());

	if (!pbuffer.valid()) {
		std::cerr << "Failed to make a graphics context" << std::endl;
		exit(-1);
	}
	//----------

	//Create the camera:
	tgt_cam = new osg::Camera;
	//Set the graphics context:
	tgt_cam->setGraphicsContext(pbuffer.get());

	tgt_cam->setClearMask( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	tgt_cam->setClearColor(osg::Vec4f(0.2f, 0.2f, 0.4f, 1.0f));

	//Attach images:
	//----------
	depth_buffer_tgt_cam_pov = new osg::Image;
	depth_buffer_tgt_cam_pov->allocateImage(tgt_width, tgt_height, 1,
			GL_DEPTH_COMPONENT, GL_FLOAT);
	tgt_cam->attach(osg::Camera::DEPTH_BUFFER, depth_buffer_tgt_cam_pov.get());

	//colour_buffer_tgt_cam_pov = new osg::Image;
	//colour_buffer_tgt_cam_pov->allocateImage( tgt_width, tgt_height, 1, GL_RGBA, GL_UNSIGNED_BYTE );
	//tgt_cam->attach( osg::Camera::COLOR_BUFFER, colour_buffer_tgt_cam_pov.get() );
	//----------

	view.setCamera(tgt_cam.get());
	view.realize();

	//Set near and far:
	near = 0.25f;
	far = 10.f;

}

void RenderPOV::render_pov(osg::ref_ptr<osg::Geode> geometry, cv::Mat* map_tgt,
		osg::Matrix3 K_tgt, osg::Matrix T_src2tgt) {

	//DEBUG TEST:
	osg::Matrix T_rot_about_x = osg::Matrix::rotate(M_PI, osg::Vec3(1, 0, 0));

	set_camera_as_pov(tgt_cam, K_tgt, map_tgt->cols, map_tgt->rows, near, far);
	//tgt_cam->setViewMatrix(T_rot_about_x*(T_src2tgt.inverse(T_src2tgt)));
	tgt_cam->setViewMatrix((T_src2tgt.inverse(T_src2tgt)) * T_rot_about_x);

	tgt_cam->setClearMask( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	tgt_cam->setClearColor(osg::Vec4f(0.2f, 0.2f, 0.4f, 1.0f));

	tgt_cam->removeChild(0, 1);
	tgt_cam->addChild(geometry);

	view.frame();
	view.frame();	//why twice?

	//Extract and convert to mm:
	convert_depth_buffer_to_mm(map_tgt->cols, map_tgt->rows,
			depth_buffer_tgt_cam_pov, map_tgt);

	//DEBUG: Save the colour frame:
	//osgDB::writeImageFile(*colour_buffer_tgt_cam_pov, "test_colour.png");

}

void RenderPOV::set_camera_as_pov(osg::Camera* camera, osg::Matrix3 K,
		int width, int height, float near, float far) {

	camera->setAllowEventFocus(false);
	camera->setProjectionMatrix(get_cam_proj_mat(K, width, height, near, far));
	//NB! disable computation of near and far
	camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	camera->setViewport(0, 0, width, height);

}

osg::Matrix RenderPOV::get_cam_proj_mat(osg::Matrix3 K, int width, int height,
		float near, float far) {

	float X = near + far;
	float Y = near * far;
	float alpha = K[0 * 3 + 0];	//K_rgb[0*3 + 0];
	float beta = K[1 * 3 + 1];	//K_rgb[1*3 + 1];
	float x_0 = K[2 * 3 + 0];	//K_rgb[0*3 + 2];
	float y_0 = K[2 * 3 + 1];	//K_rgb[1*3 + 2];

	const float pmat_val[16] = { alpha, 0, -x_0, 0, 0, -beta, -y_0, 0,//NB - changed from 0, beta, -y_0, 0,
			0, 0, X, Y, 0, 0, -1, 0 };
	//Transpose it for OpenGL:
	float pmat_transposed[16];
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			pmat_transposed[c * 4 + r] = pmat_val[r * 4 + c];
		}
	}
	osg::Matrix pmat_transposed_matrix(pmat_transposed);

	osg::Matrix proj_mat = osg::Matrix::ortho(0, width, height, 0, near, far);//NB - changed from glOrtho(0,width,0,height,near,far);
	proj_mat = pmat_transposed_matrix
			* (osg::Matrix::ortho(0, width, height, 0, near, far));	//Why this order??

	return proj_mat;
}

void RenderPOV::convert_depth_buffer_to_mm(int width, int height,
		osg::Image* osg_float, cv::Mat* cv_16_bit) {

	//Extract correctly scaled 16-bit depth image from depth buffer:
	//-------

	cv::Mat depth(height, width, CV_32F, osg_float->data(),
			width * sizeof(float));
	cv::Mat depth_32 = depth.clone();

	//Convert from normalized to m:
	double a = -(far - near) / (2.0 * far * near);
	double b = (far + near) / (2.0 * far * near);
	depth_32 = -(-1.0 / ((2 * (depth_32) - 1) * a + b));

	//Convert to 16 bit (in mm):
	depth_32.convertTo((*cv_16_bit), CV_16U, 1000.);
	cv::flip((*cv_16_bit), (*cv_16_bit), 0);

	//-------

	//NB! zero the values beyond, say 8 m (which are actualy unobserved,
	//which the depth map extraction has marked as surface at far (10m)):
	//----------------
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			if ((*cv_16_bit).at<ushort>(row, col) > 8000) {
				(*cv_16_bit).at<ushort>(row, col) = 0;
			}
		}
	}
	//----------------

}

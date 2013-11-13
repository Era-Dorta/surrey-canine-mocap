#ifndef RENDERPOV_H_
#define RENDERPOV_H_

#include <string>
#include <vector>
#include <fstream>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include <osgText/Text>
#include <osgText/Font>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/PolygonMode>
#include <osgViewer/CompositeViewer>
#include <osg/Geometry>

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#include "CudaVec.h"

//Conversion between OSG and CUDAVEC types:
extern float3x3 osgmat_2_float3x3(osg::Matrix3 input);
extern osg::Matrix float4x4_2_osgmat(float4x4 input);
extern float3 osgvec_2_float3(osg::Vec3 vec);
extern osg::Vec3 float3_2_osgvec(float3 vec);

class RenderPOV {
	public:
		RenderPOV(int tgt_width, int tgt_height);
		virtual ~RenderPOV() {
		}
		;
		void render_pov(osg::ref_ptr<osg::Geode> geometry, cv::Mat* map_tgt,
				osg::Matrix3 K_tgt, osg::Matrix T_src2tgt);

	private:
		void set_camera_as_pov(osg::Camera* camera, osg::Matrix3 K, int width,
				int height, float near, float far);
		osg::Matrix get_cam_proj_mat(osg::Matrix3 K, int width, int height,
				float near, float far);
		void convert_depth_buffer_to_mm(int width, int height,
				osg::Image* osg_float, cv::Mat* cv_16_bit);

		osg::ref_ptr<osg::Image> depth_buffer_tgt_cam_pov;
		osg::ref_ptr<osg::Image> colour_buffer_tgt_cam_pov;

		osg::ref_ptr<osg::Camera> tgt_cam;
		osgViewer::Viewer view;

		float near;
		float far;

};

#endif /* RENDERPOV_H_ */

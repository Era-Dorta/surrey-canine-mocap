#ifndef RENDERSKELETONIZATION_H
#define RENDERSKELETONIZATION_H

#include "Skeletonization3D.h"
#include "RGBDCamera.h"
#include "Node.h"
#include "MocapHeader.h"

#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/Texture2D>
#include <osg/Material>

#include "boost/shared_ptr.hpp"

#include "iostream"

using std::cout;
using std::endl;

//Type def to avoid writing this monster more than once .
typedef std::multiset<osgUtil::LineSegmentIntersector::Intersection>::iterator intersecIte;

class RenderSkeletonization {
	public:
		RenderSkeletonization();

		virtual ~RenderSkeletonization();

		void set_data(std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr_,
				osg::ref_ptr<osg::Switch> skel_vis_switch_,
				osg::ref_ptr<osg::Switch> skel_fitting_switch_);
		//Delete skeleton nodes from previous frame
		void clean_scene();

		//TODO Should not give Skeletonization3D as argument, still much
		//coupling in the code
		//Show the 2D skeleton images of each camera given a frame number
		void display_2d_skeletons(int disp_frame_no,
				Skeletonization3D& skeleton);

		//Show a 3D skeleton projection given a frame number
		void display_3d_skeleon_cloud(int disp_frame_no,
				Skeletonization3D& skeleton);

		//Show a 3D skeleton projection given a frame number
		void display_3d_merged_skeleon_cloud(int disp_frame_no,
				Skeletonization3D& skeleton);

		void evaluate_children(Node* node, MocapHeader& header,
				int current_frame);

		osg::ref_ptr<osg::MatrixTransform> createSelectionBox();
		osg::ref_ptr<osg::MatrixTransform> createSelectionBox(osg::Vec4 color);

		void display_text(std::string text, osg::Vec3 pos);

		osg::MatrixTransform* obj_belong_skel(
				osg::MatrixTransform* selected_obj);
	protected:
	private:
		osgText::Text* create_text(const osg::Vec3& pos,
				const std::string& content, float size);

		osg::Camera* create_hud_camera(double left, double right, double bottom,
				double top);

		osg::MatrixTransform* obj_belong_skel(
				osg::MatrixTransform* selected_obj,
				osg::MatrixTransform* current_node);

		void evaluate_children(Node* node, MocapHeader& header,
				osg::Group *pAddToThisGroup, int current_frame);

		void AddCylinderBetweenPoints(osg::Vec3 StartPoint, osg::Vec3 EndPoint,
				float radius, osg::Vec4 CylinderColor,
				osg::Group *pAddToThisGroup);

		//Pointer to the camera array
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr;

		//TODO Since Render Skeleton now is better design, some of the
		//osg::Switch should not longer be necessary, or at least be created
		//inside, think about creating methods to abstract them, and just
		//be like, show_skeleton()

		//Root node of all skeleton related nodes
		osg::ref_ptr<osg::Switch> skel_vis_switch;

		//Direct access to skeleton nodes, ordered by cameras
		std::vector<osg::ref_ptr<osg::Group> > skel_group_array;

		//Direct access to skeleton nodes, ordered by cameras
		std::vector<osg::ref_ptr<osg::Group> > skel_group2D_array;

		//Direct access to skeleton nodes, ordered by cameras
		std::vector<osg::ref_ptr<osg::Group> > skel_group3D_array;

		//Merge skeleton group
		osg::ref_ptr<osg::Group> merged_group;

		osg::ref_ptr<osg::Switch> skel_fitting_switch;

		bool display_merged;

		osg::Vec4 joint_color;
		osg::Vec4 bone_color;
		osg::Vec4 selection_color;
};

#endif // RENDERSKELETONIZATION_H

#ifndef RENDERSKELETONIZATION_H
#define RENDERSKELETONIZATION_H

#include "../Controller/Skeletonization3D.h"
#include "../View/RGBDCamera.h"
#include "../Model/Node.h"
#include "../Model/MocapHeader.h"
#include "../Misc/MiscUtils.h"
#include "../Controller/Fitting/SkeletonFitting.h"

#include <vector>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/Texture2D>
#include <osg/Material>

#include <boost/shared_ptr.hpp>

//Type def to avoid writing this monster more than once .
typedef std::multiset<osgUtil::LineSegmentIntersector::Intersection>::iterator intersecIte;

class RenderSkeletonization {
public:
	RenderSkeletonization(const camVecT& camera_arr_,
			osg::ref_ptr<osg::Group> render_skel_group);

	virtual ~RenderSkeletonization();

	void setup_scene();

	//Delete skeleton nodes from previous frame
	void clean_scene();

	void clean_2d_skeletons();
	void clean_3d_skeleon_cloud();
	void clean_3d_merged_skeleon_cloud();
	void clean_skeleton();
	void clean_text();

	//TODO Should not give Skeletonization3D as argument, still much
	//coupling in the code
	//Show the 2D skeleton images of each camera given a frame number
	void display_2d_skeletons(int disp_frame_no, Skeletonization3DPtr skeleton);

	//Show a 3D skeleton projection given a frame number
	void display_3d_skeleon_cloud(int disp_frame_no,
			Skeletonization3D& skeleton);

	//Show a 3D skeleton projection given a frame number
	void display_3d_merged_skeleon_cloud(int disp_frame_no,
			Skeletonization3D& skeleton);

	//Show the cloud with a different colour for each point group
	//using the labels in group
	void display_cloud(const PointCloudPtr& cloud,
			std::vector<Skeleton::Skel_Leg> group);

	void display_sphere(const osg::Vec3& position, unsigned index,
			const osg::Vec4& color = osg::Vec4(0.0, 0.0, 0.0, 0.0));

	//TODO Should not use Node, but it is much easier not to strictly follow
	//the Model-View-Controller pattern
	void display_skeleton(Node* node, MocapHeader& header, int current_frame,
			bool with_axis);

	void display_text(std::string text, osg::Vec3 pos);

	void display_line(const osg::Vec3& from, const osg::Vec3& to,
			unsigned int index);

	//If the object the user clicked on is a bone, it returns its
	//renderer transformation matrix
	osg::MatrixTransform* is_obj_bone(osg::Drawable* selected_obj);

	//Toggle cam_num cloud point
	void toggle_3d_cloud(int cam_num);

	//Toggle all cams cloud points
	void toggle_3d_cloud();

	void toggle_3d_merged_cloud();

	//Toggle cloud with labels
	void toggle_group_div();
private:
	osg::ref_ptr<osg::MatrixTransform> create_sphere(float radius,
			osg::Vec4 color);

	osg::Camera* create_hud_camera(double left, double right, double bottom,
			double top);

	osg::MatrixTransform* is_obj_bone(osg::Drawable* selected_obj,
			osg::MatrixTransform* current_node);

	void create_skeleton(Node* node, MocapHeader& header,
			osg::Group *pAddToThisGroup, int current_frame, bool with_axis);

	void update_skeleton(Node* node, MocapHeader& header, int current_frame,
			bool with_axis);

	void create_cylinder(osg::Vec3 StartPoint, osg::Vec3 EndPoint, float radius,
			osg::Vec4 CylinderColor, osg::Group *pAddToThisGroup);

	void update_cylinder(osg::Vec3 StartPoint, osg::Vec3 EndPoint, float radius,
			osg::Vec4 CylinderColor, osg::Geode *cylinder_geode);

	void add_axis_to_node(osg::Group* to_add, const osg::Matrix& trans);

	void add_sphere_to_node(float radius, osg::Vec4 color, osg::Group* to_add,
			const osg::Matrix& trans);

	//Pointer to the camera array
	const camVecT& camera_arr;

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

	osg::ref_ptr<osgText::Text> skel_edit_text;

	osg::ref_ptr<osg::Geode> axis;

	bool display_merged;
	bool skel_created;
	bool text_created;

	osg::ref_ptr<osg::Switch> skel_group_div;
};

#endif // RENDERSKELETONIZATION_H

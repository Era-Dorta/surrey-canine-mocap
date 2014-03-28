/*
 * SkeletonFitting.h
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#ifndef SKELETONCONTROLLER_H_
#define SKELETONCONTROLLER_H_

#include "../Model/Skeleton.h"
#include "../View/RenderSkeletonization.h"
#include "Skeletonization3D.h"
#include "Fitting/SkeletonFitting.h"
#include "SkeletonMixer.h"
#include "../Misc/MessageHandler.h"
#include "SkeletonState.h"
#include "Fitting/EnhancedIKSolver.h"

#include <osgUtil/LineSegmentIntersector>
#include <osg/MatrixTransform>

#include <iostream>
using std::cout;
using std::endl;

enum Fitting_State {
	EMPTY, ADD_POINTS, MOVE_POINTS, POINTS_SET
};

class SkeletonController {
public:
	SkeletonController(const camVecT& camera_arr,
			osg::ref_ptr<osg::Group> render_skel_group);

	virtual ~SkeletonController();

	Fitting_State getState() const;
	void setState(Fitting_State state);

	//Handle mouse events, to set up fitting points
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void update_dynamics(int disp_frame_no);

	//Calculate tracking information for all frames
	void generate_skeletonization();

	void setup_scene();
private:
	enum Mod_State {
		ROTATE, TRANSLATE, INV_KIN
	};

	void reset_state();
	void draw_edit_text();

	void load_skeleton_from_file(std::string file_name);
	void save_skeleton_to_file(std::string file_name);

	//Handle mouse events, to set up fitting points
	bool handle_mouse_events(const osgGA::GUIEventAdapter& ea,
			osgGA::GUIActionAdapter& aa);

	//Handle mouse events, to set up fitting points
	bool handle_keyboard_events(const osgGA::GUIEventAdapter& ea,
			osgGA::GUIActionAdapter& aa);

	//Get a useful vector to pass to the skeleton modification methods
	//from screen 2D mouse coordinates
	osg::Vec3 get_mouse_vec(int x, int y);

	//Mix manually marked up bvh skeleton files and create a new one
	void mix_skeleton_sizes();

	//Reset state to not bone modification
	void finish_bone_trans();

	//Fix bone axis after a bone modification
	void update_bones_axis();

	//Class that creates a skeleton from a given set of frames
	Skeletonization3DPtr skeletonized3D;

	SkeletonPtr skeleton;

	//Class that renders all skeleton related objects
	RenderSkeletonization skel_renderer;

	//Class with methods to modify(and create???) a skeleton to fit into a
	//cloud of points that represent a skeleton.
	SkeletonFitting skel_fitter;

	SkeletonMixer skel_mixer;

	//Class that saves current frame skeleton rotations in case the user
	//decides to restore them after a modification
	SkeletonState skel_state;

	Fitting_State state;

	int current_frame;

	bool is_point_selected;
	int selected_point_index;
	int chain_top_index;
	osg::ref_ptr<osg::MatrixTransform> selected_point;
	int last_mouse_pos_x;
	int last_mouse_pos_y;
	bool move_on_z;
	Mod_State mod_state;
	bool change_all_frames;
	bool only_root;
	unsigned int num_bones_chain;
	bool transforming_skeleton;
	bool delete_skel;
	Skeleton::Axis rotate_axis;
	bool show_joint_axis;
	bool manual_mark_up;
	float rotate_scale_factor;
	float translate_scale_factor;
	float inv_kin_scale_factor;
	float swivel_angle;
	std::vector<bool> fitting_pending;
	std::string user_home;

	MessageHandler msg_handler;

	EnhancedIKSolver enh_ik_solver;
};

#endif /* SKELETONCONTROLLER_H_ */

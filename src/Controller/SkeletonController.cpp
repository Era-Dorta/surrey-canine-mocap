/*
 * SkeletonFitting.cpp
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#include "SkeletonController.h"

SkeletonController::SkeletonController(const camVecT& camera_arr,
		osg::ref_ptr<osg::Group> render_skel_group) :
		skeletonized3D(Skeletonization3DPtr(new Skeletonization3D(camera_arr))), skeleton(
				SkeletonPtr(new Skeleton)), skel_renderer(camera_arr,
				render_skel_group), skel_fitter(skeleton, skeletonized3D,
				camera_arr), enh_ik_solver(skeleton) {
	current_frame = 0;
	last_mouse_pos_x = 0;
	last_mouse_pos_y = 0;
	num_bones_chain = 1;
	delete_skel = false;
	rotate_axis = Skeleton::X;
	show_joint_axis = false;
	manual_mark_up = false;
	rotate_scale_factor = 0.02;
	translate_scale_factor = 0.002;
	inv_kin_scale_factor = 0.0005;

	reset_state();

	//Uncomment to take the manual marked up frames
	//and mix those bone lengths to produce a averaged skeleton
	//mix_skeleton_sizes();
}

SkeletonController::~SkeletonController() {
}

bool SkeletonController::handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa) {

	//If true, then we handled the event and do not want further interaction
	if (handle_mouse_events(ea, aa)) {
		return true;
	}

	handle_keyboard_events(ea, aa);

	return false;
}

void SkeletonController::load_skeleton_from_file(std::string file_name) {
	skel_renderer.clean_skeleton();
	skel_renderer.clean_3d_merged_skeleon_cloud();

	skeleton->load_from_file(file_name);

	reset_state();
	//Reset the fitting vector
	fitting_pending.assign(fitting_pending.size(), true);

	update_dynamics(current_frame);
}

void SkeletonController::save_skeleton_to_file(std::string file_name) {
	skeleton->save_to_file(file_name);
}

void SkeletonController::reset_state() {
	state = MOVE_POINTS;
	is_point_selected = false;
	selected_point_index = 0;
	chain_top_index = 0;
	move_on_z = false;
	mod_state = INV_KIN;
	change_all_frames = false;
	transforming_skeleton = false;
	only_root = false;
	swivel_angle = 0.0;
}

void SkeletonController::update_dynamics(int disp_frame_no) {
	//TODO This recreates the scene over and over, should just be some updating
	//not creating everything from scratch
	//Skeleton, text and 3dmerged cloud are not recreated every frame anymore
	current_frame = disp_frame_no;
	skeleton->set_current_frame(current_frame);
	skel_fitter.calculate_for_frame(current_frame);

	//skel_renderer.clean_2d_skeletons();
	//skel_renderer.display_2d_skeletons(current_frame, skeletonized3D);

	//skel_renderer.clean_3d_skeleon_cloud();
	//skel_renderer.display_3d_skeleon_cloud(current_frame, *skeletonized3D);

	//skel_renderer.display_3d_merged_skeleon_cloud(current_frame,
	//		(*skeletonized3D));

	skel_renderer.display_cloud(
			skeletonized3D->get_merged_3d_projection(current_frame),
			skel_fitter.getLabels());

	//skel_renderer.display_sphere(skel_fitter.get_paw(Skeleton::Front_Right), 0);
	//skel_renderer.display_sphere(skel_fitter.get_paw(Skeleton::Front_Left), 1);
	//skel_renderer.display_sphere(skel_fitter.get_paw(Skeleton::Back_Right), 2);
	//skel_renderer.display_sphere(skel_fitter.get_paw(Skeleton::Back_Left), 3);
	//skel_renderer.display_sphere(skel_fitter.get_new_pos()[0], 0);
	//skel_renderer.display_sphere(skel_fitter.get_new_pos()[1], 1);
	//skel_renderer.display_sphere(skel_fitter.get_new_pos()[2], 2);
	//skel_renderer.display_line(osg::Vec3(0,0,0) * current_frame, osg::Vec3(1,1,1), 3);
	//skel_renderer.display_line(osg::Vec3(1,1,1) * current_frame, osg::Vec3(2,0,1), 4);
	//skel_renderer.display_line(skel_fitter.get_new_lines()[0], skel_fitter.get_new_lines()[1], 3);
	//skel_renderer.display_line(skel_fitter.get_new_lines()[2], skel_fitter.get_new_lines()[3], 4);

	if (skeleton->isSkelLoaded()) {
		//Do the fitting only once per frame, then let the user modify the
		//fitted skeleton if they wish to do so
		if (fitting_pending.at(current_frame)) {
			skel_fitter.fit_skeleton_to_cloud();
			fitting_pending.at(current_frame) = false;
		}

		if (delete_skel) {
			skel_renderer.clean_skeleton();
		}
		skel_renderer.display_skeleton(skeleton->get_root(),
				skeleton->get_header(), current_frame, show_joint_axis);
		draw_edit_text();
	}
}

void SkeletonController::generate_skeletonization() {
	skeletonized3D->generate_skeletonization();
	fitting_pending.resize(skeletonized3D->get_n_frames(), true);
}

void SkeletonController::setup_scene() {
	skel_renderer.setup_scene();
}

Fitting_State SkeletonController::getState() const {
	return state;
}

void SkeletonController::setState(Fitting_State state) {
	this->state = state;
}

void SkeletonController::draw_edit_text() {
	if (is_point_selected) {
		std::string edit_text =
				"v(finish) b(rot) n(axis) m(frames) ,(root) +-(n_bones)\n";
		edit_text += "Editing ";
		switch (mod_state) {
		case ROTATE:
			edit_text += "rotating ";
			break;
		case TRANSLATE:
			if (change_all_frames && !only_root) {
				edit_text += "resizing ";
			} else {
				edit_text += "translating ";
			}
			break;
		case INV_KIN:
			edit_text += "inv kin ";
			break;
		}

		switch (rotate_axis) {
		case Skeleton::X:
			edit_text += "red ";
			break;
		case Skeleton::Y:
			edit_text += "green ";
			break;
		case Skeleton::Z:
			edit_text += "blue ";
			break;
		}
		if (change_all_frames) {
			edit_text += "all frames ";
		} else {
			edit_text += "current frames ";
		}
		if (only_root) {
			edit_text += "root ";
		} else {
			edit_text += "bone ";
		}

		edit_text += "n_bones ";

		std::ostringstream oss;
		oss << num_bones_chain;
		edit_text += oss.str();

		skel_renderer.display_text(edit_text, osg::Vec3(600.0f, 50.0f, 0.0f));
	}
}

bool SkeletonController::handle_mouse_events(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa) {
	//If the user release the left mouse button while pressing control
	//then use the line intersector to mark a point of the skeleton
	if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE
			&& ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON
			&& (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {

		osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);

		if (viewer) {
			osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
					new osgUtil::LineSegmentIntersector(
							osgUtil::Intersector::WINDOW, ea.getX(), ea.getY());

			osgUtil::IntersectionVisitor iv(intersector.get());
			iv.setTraversalMask(~0x1);
			viewer->getCamera()->accept(iv);

			if (intersector->containsIntersections()) {
				switch (state) {
				case MOVE_POINTS: {
					if (!is_point_selected) {

						intersecIte result =
								intersector->getIntersections().begin();

						osg::Drawable* selected_obj = result->drawable;
						selected_point = skel_renderer.is_obj_bone(
								selected_obj);
						if (selected_point) {
							is_point_selected = true;
							transforming_skeleton = true;
							selected_point_index = skeleton->get_node_index(
									selected_point);
							chain_top_index = selected_point_index
									- num_bones_chain;
							if (chain_top_index < 0) {
								chain_top_index = 0;
							}
							skeleton->toggle_color(selected_point_index);
							delete_skel = true;
							skel_state.save_state(skeleton, current_frame);
							update_dynamics(current_frame);
						}
					}
					break;
				}
				case EMPTY:
				case ADD_POINTS:
				case POINTS_SET:
					break;
				}
			}
		}
		last_mouse_pos_x = ea.getX();
		last_mouse_pos_y = ea.getY();
	}

	if (is_point_selected && ea.getEventType() == osgGA::GUIEventAdapter::DRAG
			&& transforming_skeleton) {

		osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);

		if (viewer) {
			osg::Vec3 move_axis = get_mouse_vec(ea.getX(), ea.getY());

			switch (mod_state) {
			case ROTATE:
				if (!change_all_frames) {
					if (!only_root) {
						skeleton->rotate_joint(selected_point_index, move_axis);
					}
				} else {
					if (only_root) {
						skeleton->rotate_root_all_frames(move_axis);
					}
				}
				break;
			case TRANSLATE:
				if (only_root) {
					if (!change_all_frames) {
						skeleton->translate_root(move_axis);
					} else {
						skeleton->translate_root_all_frames(move_axis);
					}
				} else {
					if (change_all_frames) {
						skeleton->change_bone_length_all_frames(
								selected_point_index, move_axis);
					}
				}
				break;
			case INV_KIN:
				if (!change_all_frames) {
					if (!only_root) {
						//move_axis.set(0, 0.15, -0.15);
						//Simple solver
						//enh_ik_solver.solve_chain(
						//		selected_point_index - num_bones_chain,
						//		selected_point_index, make_float3(move_axis._v),
						//		current_frame)) {
						//
						enh_ik_solver.solve_chain_keep_next_bone_pos(
								chain_top_index, selected_point_index,
								make_float3(move_axis._v), current_frame);
					} else {
						skeleton->rotate_two_bones_keep_end_pos_aim(
								selected_point_index, swivel_angle);
					}
				}
				break;
			}

			update_dynamics(current_frame);

			last_mouse_pos_x = ea.getX();
			last_mouse_pos_y = ea.getY();
			return true;
		}
	}

	if (is_point_selected
			&& ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON
			&& ea.getEventType() == osgGA::GUIEventAdapter::PUSH
			&& transforming_skeleton) {
		last_mouse_pos_x = ea.getX();
		last_mouse_pos_y = ea.getY();
	}

	return false;
}

bool SkeletonController::handle_keyboard_events(
		const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
	switch (ea.getEventType()) {
	case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		//Toggle skel cam 1 visibility:
		case osgGA::GUIEventAdapter::KEY_Q:
			skel_renderer.toggle_3d_cloud(0);
			update_dynamics(current_frame);
			break;

			//Toggle skel cam 2 visibility:
		case osgGA::GUIEventAdapter::KEY_W:
			skel_renderer.toggle_3d_cloud(1);
			update_dynamics(current_frame);
			break;

			//Toggle skel cam 3 visibility:
		case osgGA::GUIEventAdapter::KEY_E:
			skel_renderer.toggle_3d_cloud(2);
			update_dynamics(current_frame);
			break;

			//Toggle all skel cam visibility:
		case osgGA::GUIEventAdapter::KEY_R:
			//skel_renderer.toggle_3d_cloud();
			skel_renderer.toggle_group_div();
			update_dynamics(current_frame);
			break;

			//Toggle merged skeleton visibility:
		case osgGA::GUIEventAdapter::KEY_T:
			skel_renderer.toggle_3d_merged_cloud();
			update_dynamics(current_frame);
			break;
		case osgGA::GUIEventAdapter::KEY_V:
			if (is_point_selected) {
				finish_bone_trans();
			}
			break;
		case osgGA::GUIEventAdapter::KEY_B:
			if (is_point_selected) {
				switch (mod_state) {
				case ROTATE:
					mod_state = TRANSLATE;
					break;
				case TRANSLATE:
					mod_state = INV_KIN;
					break;
				case INV_KIN:
					mod_state = ROTATE;
					break;
				}
				update_bones_axis();
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_N:
			if (is_point_selected) {
				switch (rotate_axis) {
				case Skeleton::X:
					rotate_axis = Skeleton::Y;
					break;
				case Skeleton::Y:
					rotate_axis = Skeleton::Z;
					break;
				case Skeleton::Z:
					rotate_axis = Skeleton::X;
					break;
				}
				update_bones_axis();
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_M:
			if (is_point_selected) {
				change_all_frames = !change_all_frames;
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_Comma:
			if (is_point_selected) {
				only_root = !only_root;
				update_bones_axis();
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_Plus:
		case osgGA::GUIEventAdapter::KEY_KP_Add:
			if (is_point_selected) {
				num_bones_chain++;
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_Minus:
		case osgGA::GUIEventAdapter::KEY_KP_Subtract:
			if (is_point_selected && num_bones_chain > 0) {
				num_bones_chain--;
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_G:
			skel_renderer.clean_skeleton();
			show_joint_axis = !show_joint_axis;
			update_dynamics(current_frame);
			break;
		case osgGA::GUIEventAdapter::KEY_Control_L:
			if (is_point_selected) {
				transforming_skeleton = !transforming_skeleton;
				update_dynamics(current_frame);
			}
			break;
			//Load skeleton from a file:
		case osgGA::GUIEventAdapter::KEY_L:
			cout << "loading skeleton file" << endl;
			load_skeleton_from_file(
			//		"/home/cvssp/misc/m04701/workspace/data/bvh/dog_resized.bvh");
			//"/home/cvssp/misc/m04701/workspace/data/bvh/Dog_modelling.bvh");
			//"/home/cvssp/misc/m04701/workspace/data/bvh/Dog_modelling_centered.bvh");
			//"/home/cvssp/misc/m04701/workspace/data/bvh/vogueBLong.bvh");
			//"/home/cvssp/misc/m04701/workspace/data/bvh/dog_manual_mark_up29.bvh");
			//"/home/cvssp/misc/m04701/workspace/data/bvh/4bones.bvh");
					"/home/cvssp/misc/m04701/workspace/data/bvh/out2.bvh");
			break;

			//Save skeleton to file:
		case osgGA::GUIEventAdapter::KEY_K:
			if (!manual_mark_up) {
				save_skeleton_to_file(
						"/home/cvssp/misc/m04701/workspace/data/bvh/out2.bvh");
			} else {
				std::string file_name =
						"/home/cvssp/misc/m04701/workspace/data/bvh/dog_manual_mark_up";
				std::stringstream out;
				out << current_frame;
				file_name += out.str();
				file_name += ".bvh";
				save_skeleton_to_file(file_name);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_BackSpace:
			if (is_point_selected) {
				skel_state.restore_state(skeleton, current_frame);
				finish_bone_trans();
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return false;
}

osg::Vec3 SkeletonController::get_mouse_vec(int x, int y) {
	osg::Vec3 mouse_vec;

	switch (rotate_axis) {
	case Skeleton::X:
		mouse_vec.set(last_mouse_pos_y - y, 0.0, 0.0);
		break;
	case Skeleton::Y:
		mouse_vec.set(0.0, last_mouse_pos_y - y, 0.0);
		break;
	case Skeleton::Z:
		mouse_vec.set(0.0, 0.0, y - last_mouse_pos_y);
		break;
	}

	switch (mod_state) {
	case ROTATE:
		mouse_vec = mouse_vec * rotate_scale_factor;
		break;
	case TRANSLATE:
		mouse_vec = mouse_vec * translate_scale_factor;
		break;
	case INV_KIN:
		if (!only_root) {
			mouse_vec =
					mouse_vec * inv_kin_scale_factor
							+ skeleton->get_node(selected_point_index)->get_end_bone_global_pos(
									current_frame);
		} else {
			if (last_mouse_pos_y - y > 0) {
				swivel_angle = 0.02;
			} else {
				swivel_angle = -0.02;
			}
		}
		break;
	}

	return mouse_vec;
}

void SkeletonController::mix_skeleton_sizes() {
	int start_frame = 29, end_frame = 29;
	std::vector<std::string> file_names;
	for (int i = start_frame; i <= end_frame; i++) {
		std::string file_name =
				"/home/cvssp/misc/m04701/workspace/data/bvh/dog_manual_mark_up";
		std::stringstream out;
		out << i;
		file_name += out.str();
		file_name += ".bvh";
		file_names.push_back(file_name);
	}

	std::string file_name =
			"/home/cvssp/misc/m04701/workspace/data/bvh/Dog_modelling.bvh";
	skel_mixer.init(file_name, file_names);
	skel_mixer.mix();
	file_name =
			"/home/cvssp/misc/m04701/workspace/data/bvh/dog_manual_mark_up_mixed_legs_impr.bvh";
	skel_mixer.save_file(file_name);
}

void SkeletonController::finish_bone_trans() {
	update_bones_axis();
	skeleton->toggle_color(selected_point_index);
	reset_state();
	skel_renderer.clean_text();
	update_dynamics(current_frame);
	delete_skel = false;
}

void SkeletonController::update_bones_axis() {
	for (int i = chain_top_index; i <= selected_point_index; i++) {
		skeleton->get_node(i)->optimize_rotation(current_frame);
	}
	Node* node = skeleton->get_node(selected_point_index);
	node->optimize_rotation(current_frame);
}

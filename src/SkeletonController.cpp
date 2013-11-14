/*
 * SkeletonFitting.cpp
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#include "SkeletonController.h"

SkeletonController::SkeletonController() :
			state(MOVE_POINTS), is_point_selected(false),
			selected_point_index(0), current_frame(0), last_mouse_pos_x(0),
			last_mouse_pos_y(0), move_on_z(false), translate_root(false),
			change_all_frames(false), inter_number(0) {
	unselected_color = osg::Vec4(0.5f, 0.5f, 0.5f, 1.0); //Grey
	selected_color = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0); //White
}

SkeletonController::~SkeletonController() {
	// TODO Auto-generated destructor stub
}

void SkeletonController::set_data(osg::ref_ptr<osg::Switch> skel_fitting_switch,
		std::vector<boost::shared_ptr<RGBD_Camera> > camera_arr,
		osg::ref_ptr<osg::Switch> skel_vis_switch) {
	skel_renderer.set_data(camera_arr, skel_vis_switch, skel_fitting_switch);
	skeletonized3D.set_cameras(camera_arr);
}

bool SkeletonController::handle(const osgGA::GUIEventAdapter& ea,
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
						if (inter_number
								>= intersector->getIntersections().size()) {
							inter_number = 0;
						}
						intersecIte result =
								intersector->getIntersections().begin();
						for (unsigned int i = 0; i < inter_number; i++) {
							result++;
						}
						osg::MatrixTransform* selected_obj =
								dynamic_cast<osg::MatrixTransform*>(result->drawable->getParent(
										0)->getParent(0));
						if (selected_obj) {
							selected_point_color =
									skel_renderer.obj_belong_skel(selected_obj);
							if (selected_point_color) {
								is_point_selected = true;
								osg::ref_ptr<osg::MatrixTransform> selected_point =
										static_cast<osg::MatrixTransform*>(selected_point_color->getParent(
												0));
								selected_point_index = skeleton.get_node(
										selected_point);
								skeleton.change_color(selected_point_index,
										selected_color);
								update_dynamics(current_frame);
							}
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
			&& (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {

		osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);

		if (viewer) {
			osg::Vec3 move_axis;
			if (move_on_z) {
				move_axis.set(0.0, 0.0, ea.getY() - last_mouse_pos_y);
			} else {
				move_axis.set(ea.getX() - last_mouse_pos_x,
						last_mouse_pos_y - ea.getY(), 0.0);
			}

			if (!translate_root) {
				if (!change_all_frames) {
					skeleton.rotate_joint(selected_point_index, move_axis);
				} else {
					skeleton.rotate_every_frame(move_axis);
				}
			} else {
				if (!change_all_frames) {
					skeleton.translate_root(move_axis);
				} else {
					skeleton.translate_every_frame(move_axis);
				}
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
			&& (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {
		last_mouse_pos_x = ea.getX();
		last_mouse_pos_y = ea.getY();
	}

	if (is_point_selected
			&& ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON
			&& (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {
		if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
			move_on_z = true;
			last_mouse_pos_x = ea.getX();
			last_mouse_pos_y = ea.getY();
		}
		if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE) {
			move_on_z = false;
			last_mouse_pos_x = ea.getX();
			last_mouse_pos_y = ea.getY();
		}
	}

	switch (ea.getEventType()) {
	case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		case osgGA::GUIEventAdapter::KEY_V:
			if (is_point_selected) {
				is_point_selected = false;
				move_on_z = false;
				translate_root = false;
				change_all_frames = false;
				skeleton.change_color(selected_point_index, unselected_color);
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_B:
			if (is_point_selected) {
				translate_root = !translate_root;
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_N:
			if (is_point_selected) {
				change_all_frames = !change_all_frames;
				update_dynamics(current_frame);
			}
			break;
		case osgGA::GUIEventAdapter::KEY_M:
			inter_number++;
			if (inter_number == 3) {
				inter_number = 0;
			}
			update_dynamics(current_frame);
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

void SkeletonController::load_skeleton_from_file(std::string file_name) {

	skeleton.load_from_file(file_name);

	reset_state();

	update_dynamics(current_frame);
}

void SkeletonController::save_skeleton_to_file(std::string file_name) {
	skeleton.save_to_file(file_name);
}

void SkeletonController::reset_state() {
	state = MOVE_POINTS;
}

void SkeletonController::draw_complete_skeleton() {
	if (skeleton.isSkelLoaded()) {
		skel_renderer.evaluate_children(skeleton.get_root(),
				skeleton.get_header(), current_frame);
	}
}

void SkeletonController::update_dynamics(int disp_frame_no) {
	//TODO This recreates the scene over and over, should just be some updating
	//not creating everything from scratch
	current_frame = disp_frame_no;
	reset_state();
	skeleton.set_current_frame(current_frame);

	skel_renderer.clean_scene();
	skel_renderer.display_3d_skeleon_cloud(disp_frame_no, skeletonized3D);
	skel_renderer.display_3d_merged_skeleon_cloud(disp_frame_no,
			skeletonized3D);

	draw_edit_text();
	draw_complete_skeleton();
}

Fitting_State SkeletonController::getState() const {
	return state;
}

void SkeletonController::setState(Fitting_State state) {
	this->state = state;
}

void SkeletonController::draw_edit_text() {
	if (is_point_selected) {
		std::string edit_text;
		edit_text += "Editing ";
		if (change_all_frames) {
			edit_text += "all frames ";
		} else {
			edit_text += "current frames ";
		}
		if (translate_root) {
			edit_text += "translating ";
		} else {
			edit_text += "rotating ";
		}
		edit_text += "intersect num ";

		std::stringstream out;
		out << inter_number;
		edit_text += out.str();
		skel_renderer.display_text(edit_text, osg::Vec3(600.0f, 20.0f, 0.0f));
		edit_text = "v(finish) b(rot) n(frames) m(inter)";
		skel_renderer.display_text(edit_text, osg::Vec3(600.0f, 50.0f, 0.0f));
	}
}

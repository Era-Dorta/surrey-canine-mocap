/*
 * SkeletonFitting.cpp
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#include "SkeletonController.h"

SkeletonController::SkeletonController() :
			state(ADD_POINTS), point_selected(false), selected_point_index(0),
			current_frame(0) {
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
				case ADD_POINTS: {
					intersecIte result;
					result = intersector->getIntersections().begin();

					//Save it as a joint
					osg::Vec3 aux = skel_renderer.add_sphere(result);
					skel_fitting.add_joint(aux);
					//If the skeleton is full of joints then change state and
					//save current state of the skeleton to output file
					if (skel_fitting.skeleton_full()) {
						state = MOVE_POINTS;
						//update_dynamics(current_frame);
					}
					break;
				}
				case MOVE_POINTS: {
					//Uncomment if not able to add points in given frame
					//if (!skel_fitting.skeleton_full()) {
					//	state = ADD_POINTS;
					//}
					if (!point_selected) {
						intersecIte result =
								intersector->getIntersections().begin();
						osg::MatrixTransform* selected_obj =
								dynamic_cast<osg::MatrixTransform*>(result->drawable->getParent(
										0)->getParent(0));
						if (selected_obj) {
							selected_point_index =
									skel_renderer.obj_belong_skel(selected_obj);
							if (selected_point_index >= 0) {
								point_selected = true;
								selected_point = selected_obj;
								skel_renderer.change_colour_when_selected(
										selected_point, point_selected);
							}
						}
					} else {
						intersecIte result =
								intersector->getIntersections().begin();

						point_selected = false;
						skel_renderer.change_colour_when_selected(
								selected_point, point_selected);
						osg::Vec3 aux = skel_renderer.move_sphere(result,
								selected_point);
						skel_fitting.move_joint(selected_point_index, aux);
						update_dynamics(current_frame);
					}
					break;
				}
				case EMPTY:
				case POINTS_SET:
					break;
				}
			}
		}
	}
	return false;
}

void SkeletonController::load_skeleton_from_file(std::string file_name) {

	skel_fitting.load_from_file(file_name);

	reset_state();

	update_dynamics(current_frame);
}

void SkeletonController::save_skeleton_to_file(std::string file_name) {
	skel_fitting.save_to_file(file_name);
}

void SkeletonController::reset_state() {
	point_selected = false;
	selected_point_index = 0;
	if (skel_fitting.skeleton_full()) {
		state = MOVE_POINTS;
	} else {
		state = ADD_POINTS;
	}
}

void SkeletonController::draw_complete_skeleton() {
	osg::Vec3 bone_start_position, bone_end_position;

	skel_renderer.draw_joints(skel_fitting.getJointArray());

	if (skel_fitting.skeleton_full()) {
		for (unsigned int i = 0; i < skel_fitting.get_num_bones(); i++) {
			skel_fitting.get_bone(i, bone_start_position, bone_end_position);
			skel_renderer.draw_bone(bone_start_position, bone_end_position);
		}
	}
}

void SkeletonController::update_dynamics(int disp_frame_no) {
	current_frame = disp_frame_no;
	reset_state();

	skel_renderer.clean_scene();
	skel_renderer.display_3d_skeleon_cloud(disp_frame_no, skeletonized3D);
	skel_renderer.display_3d_merged_skeleon_cloud(disp_frame_no,
			skeletonized3D);

	skel_fitting.set_current_frame(current_frame);
	draw_complete_skeleton();
}

Fitting_State SkeletonController::getState() const {
	return state;
}

void SkeletonController::setState(Fitting_State state) {
	this->state = state;
}

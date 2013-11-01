/*
 * SkeletonFitting.cpp
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#include "SkeletonFitController.h"

SkeletonFitController::SkeletonFitController() :
			state(ADD_POINTS), point_selected(false), selected_point_index(0) {
	joint_colour = osg::Vec4(0.0f, 0.0f, 0.0f, 1.0); //Black
	selection_colour = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0); //White
}

SkeletonFitController::~SkeletonFitController() {
	// TODO Auto-generated destructor stub
}

void SkeletonFitController::set_data(osg::ref_ptr<osg::Switch> root_node) {
	skel_fitting_switch = root_node;
	skel_fitting.load_from_file();
	for (unsigned int i = 0; i < skel_fitting.get_num_joints(); i++) {
		osg::Vec3 joint_position = skel_fitting.get_joint(i);
		osg::ref_ptr<osg::MatrixTransform> selectionBox = createSelectionBox();
		selectionBox->setMatrix(
				osg::Matrix::scale(0.01, 0.01, 0.01)
						* osg::Matrix::translate(joint_position));

		skel_fitting_switch->addChild(selectionBox.get(), true);
	}
}

bool SkeletonFitController::handle(const osgGA::GUIEventAdapter& ea,
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
					std::multiset<osgUtil::LineSegmentIntersector::Intersection>::iterator result;
					result = intersector->getIntersections().begin();

					osg::BoundingBox bb = result->drawable->getBound();
					osg::Vec3 worldCenter = bb.center()
							* osg::computeLocalToWorld(result->nodePath);

					osg::ref_ptr<osg::MatrixTransform> selectionBox =
							createSelectionBox();
					selectionBox->setMatrix(
							osg::Matrix::scale(bb.xMax() + 0.005 - bb.xMin(),
									bb.yMax() + 0.005 - bb.yMin(),
									bb.zMax() + 0.005 - bb.zMin())
									* osg::Matrix::translate(worldCenter));

					skel_fitting_switch->addChild(selectionBox.get(), true);

					//Get global coordinates of the point
					osg::Vec3 aux = osg::Vec3() * selectionBox->getMatrix();
					//Save it as a joint
					skel_fitting.add_joint(aux);
					//If the skeleton is full of joints then change state and
					//save current state of the skeleton to output file
					if (skel_fitting.skeleton_full()) {
						state = MOVE_POINTS;
						skel_fitting.save_to_file();
					}
					break;
				}
				case MOVE_POINTS: {
					if (!point_selected) {
						std::multiset<
								osgUtil::LineSegmentIntersector::Intersection>::iterator result;
						result = intersector->getIntersections().begin();
						osg::MatrixTransform* selected_obj =
								dynamic_cast<osg::MatrixTransform*>(result->drawable->getParent(
										0)->getParent(0));
						if (selected_obj) {
							for (unsigned int i = 0;
									i < skel_fitting_switch->getNumChildren();
									i++) {
								if (selected_obj
										== skel_fitting_switch->getChild(i)) {
									point_selected = true;
									selected_point = selected_obj;
									selected_point_index = i;
									change_colour_when_selected();
									//TODO Change colour to show that it was selected
									//skel_fitting_switch->setValue(i, !skel_fitting_switch->getValue(i));
								}
							}
						}
					} else {
						std::multiset<
								osgUtil::LineSegmentIntersector::Intersection>::iterator result;
						result = intersector->getIntersections().begin();

						osg::BoundingBox bb = result->drawable->getBound();
						osg::Vec3 worldCenter = bb.center()
								* osg::computeLocalToWorld(result->nodePath);

						selected_point->setMatrix(
								osg::Matrix::scale(
										bb.xMax() + 0.005 - bb.xMin(),
										bb.yMax() + 0.005 - bb.yMin(),
										bb.zMax() + 0.005 - bb.zMin())
										* osg::Matrix::translate(worldCenter));
						point_selected = false;
						change_colour_when_selected();
						osg::Vec3 aux = osg::Vec3()
								* selected_point->getMatrix();
						skel_fitting.move_joint(selected_point_index, aux);
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

void SkeletonFitController::change_colour_when_selected() {
	osg::ref_ptr<osg::ShapeDrawable> box_shape;
	osg::ref_ptr<osg::Geode> box_geode;

	box_geode = static_cast<osg::Geode*>(selected_point->getChild(0));
	box_shape = static_cast<osg::ShapeDrawable*>(box_geode->getDrawable(0));
	if (point_selected) {
		box_shape->setColor(selection_colour);
	} else {
		box_shape->setColor(joint_colour);
	}
}

osg::ref_ptr<osg::MatrixTransform> SkeletonFitController::createSelectionBox() {

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::ShapeDrawable> box_shape;
	box_shape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.0f));
	box_shape->setColor(joint_colour);

	geode->addDrawable(box_shape);
	osg::ref_ptr<osg::MatrixTransform> selectionBox = new osg::MatrixTransform;

	selectionBox->addChild(geode.get());

	return selectionBox;
}

Fitting_State SkeletonFitController::getState() const {
	return state;
}

void SkeletonFitController::setState(Fitting_State state) {
	this->state = state;
}
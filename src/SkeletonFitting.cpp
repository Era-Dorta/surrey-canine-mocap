/*
 * SkeletonFitting.cpp
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"

SkeletonFitting::SkeletonFitting() {
	// TODO Auto-generated constructor stub

}

SkeletonFitting::~SkeletonFitting() {
	// TODO Auto-generated destructor stub
}

void SkeletonFitting::set_data(osg::ref_ptr<osg::Switch> root_node) {
	skel_fitting_switch = root_node;
	skel_fitting_switch->addChild(getOrCreateSelectionBox(), false);
}

bool SkeletonFitting::handle(const osgGA::GUIEventAdapter& ea,
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
				std::multiset<osgUtil::LineSegmentIntersector::Intersection>::iterator result;
				result = intersector->getIntersections().begin();

				osg::BoundingBox bb = result->drawable->getBound();
				osg::Vec3 worldCenter = bb.center()
						* osg::computeLocalToWorld(result->nodePath);

				_selectionBox->setMatrix(
						osg::Matrix::scale(bb.xMax() + 0.01 - bb.xMin(),
								bb.yMax() + 0.01 - bb.yMin(),
								bb.zMax() + 0.01 - bb.zMin())
								* osg::Matrix::translate(worldCenter));
				//If an object is selected then show the box
				skel_fitting_switch->setValue(0, true);
			} else {
				//If no object is selected hide the box
				skel_fitting_switch->setValue(0, false);
			}
		}
	}
	return false;
}

osg::Node* SkeletonFitting::getOrCreateSelectionBox() {

	if (!_selectionBox) {
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(
				new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.0f)));
		_selectionBox = new osg::MatrixTransform;

		_selectionBox->addChild(geode.get());
	}
	return _selectionBox.get();
}


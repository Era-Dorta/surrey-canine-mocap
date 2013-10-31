/*
 * SkeletonFitting.cpp
 *
 *  Created on: 30 Oct 2013
 *      Author: m04701
 */

#include "SkeletonFitting.h"

SkeletonFitting::SkeletonFitting() :
			state(ADD_POINTS),
			points_added(0) {

}

SkeletonFitting::~SkeletonFitting() {
	// TODO Auto-generated destructor stub
}

void SkeletonFitting::set_data(osg::ref_ptr<osg::Switch> root_node) {
	skel_fitting_switch = root_node;
	//skel_fitting_switch->addChild(getOrCreateSelectionBox(), false);
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
				switch(state)
				{
				case ADD_POINTS:{

					result = intersector->getIntersections().begin();

					osg::BoundingBox bb = result->drawable->getBound();
					osg::Vec3 worldCenter = bb.center()
							* osg::computeLocalToWorld(result->nodePath);

					osg::ref_ptr<osg::MatrixTransform> selectionBox = createSelectionBox();
					selectionBox->setMatrix(
							osg::Matrix::scale(bb.xMax() + 0.01 - bb.xMin(),
									bb.yMax() + 0.01 - bb.yMin(),
									bb.zMax() + 0.01 - bb.zMin())
									* osg::Matrix::translate(worldCenter));

					skel_fitting_switch->addChild(selectionBox.get(), true);
					points_added++;
					if(points_added > 1){
						state = MOVE_POINTS;
					}
					break;
				}
				case MOVE_POINTS:{
					result = intersector->getIntersections().begin();
					osg::MatrixTransform* selected_obj = dynamic_cast<osg::MatrixTransform*>(result->drawable->getParent(0)->getParent(0));
					if(selected_obj){
						for(unsigned int i = 0; i < skel_fitting_switch->getNumChildren(); i++ ){
							if(selected_obj == skel_fitting_switch->getChild(i)){
								skel_fitting_switch->setValue(i, !skel_fitting_switch->getValue(i));
							}
						}
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

osg::ref_ptr<osg::MatrixTransform> SkeletonFitting::createSelectionBox() {

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(
				new osg::ShapeDrawable(new osg::Box(osg::Vec3(), 1.0f)));
		osg::ref_ptr<osg::MatrixTransform> selectionBox = new osg::MatrixTransform;

		selectionBox->addChild(geode.get());

	return selectionBox;
}

Fitting_State SkeletonFitting::getState() const {
	return state;
}

void SkeletonFitting::setState(Fitting_State state) {
	this->state = state;
}

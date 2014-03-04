//============================================================================
// Name        : Multi_Cam_Viewer.cpp
// Author      :
// Version     :
// Copyright   :
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "Controller/MultiCamViewer.h"

#include <iostream>
using namespace std;

int main(int argc, char** argv) {

//	//DEBUG TEST: Test SurfelModel:
//	//------------------------------------------
//
//	SurfelModel sm;
//
//	osg::ref_ptr<osg::Group> root = new osg::Group;
//	osg::ref_ptr<osg::Geode> axis = create_axis();
//	root->addChild(axis.get());
//
//	root->addChild(sm.surfel_geode);
//
//	osgViewer::Viewer viewer;
//
//	viewer.setUpViewInWindow(100,100,1024,768,0);
//	viewer.setSceneData(root);
//	return viewer.run();
//
//	//------------------------------------------

	std::string path("---");

	//Read command line arguments:
	if (argc != 2) {
		//printf("Usage: %s [dataset path]", argv[0]);
		//Set default path when no arguments are given
		path =
				"/home/cvssp/misc/m04701/workspace/data/UniS_Fitzpatrick_04_July_2013/RGBD_Cap_2013.07.04_12.58.30_Labrador";
		//exit(-1);
	} else {
		path = argv[1];
	}

	MultiCamViewer mv(path);

	mv.run_viewer();

	return 0;
}

/*
 * MultiCamViewer.cpp
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#include "MultiCamViewer.h"

using std::cout;
using std::endl;

MultiCamViewer::MultiCamViewer(std::string path) :
			win_width(1280),
			win_height(720),
			paused(true),
			with_colour(false),
			frame_period_s(1.0 / 30.0), //30fps
			last_frame_tick_count(0),
			_dataset_path(path),
			scene_root(new osg::Group),
			rgb_render_interactive_view(new osg::Image),
			cam_vis_switch(new osg::Switch),
			skel_vis_switch(new osg::Switch),
			skel_fitting_switch(new osg::Switch),
			frame_num_text(
					create_text(osg::Vec3(20.0f, 20.0f, 0.0f),
							"Frame range XXX-XXX, displaying frame: XXX",
							18.0f)), alpha(0.f) {
	//Get the list of cameras and construct camera objects for them:
	std::vector<std::string> cam_names;
	get_dir_names(path, &cam_names);
	for (unsigned int i = 0; i < cam_names.size(); i++) {
		boost::shared_ptr<RGBD_Camera> cam(
				new RGBD_Camera(_dataset_path, cam_names[i]));
		camera_arr.push_back(cam);
	}

	//Get the range of frames that all cameras have:
	begin_frame_no = -1e9;
	end_frame_no = 1e9;
	for (unsigned int i = 0; i < camera_arr.size(); i++) {

		if (begin_frame_no < camera_arr[i]->get_first_frame_num()) {
			begin_frame_no = camera_arr[i]->get_first_frame_num();
		}

		if (end_frame_no > camera_arr[i]->get_last_frame_num()) {
			end_frame_no = camera_arr[i]->get_last_frame_num();
		}

	}
	//DEBUG:
	//cout << "begin_frame_no: " << begin_frame_no << " end_frame_no: " << end_frame_no << endl;

	//Set currently displayed frame to beginning:
	disp_frame_no = begin_frame_no;

	//skel_renderer.set_data();
	skel_controller.set_data(skel_fitting_switch, camera_arr, skel_vis_switch);
}

MultiCamViewer::~MultiCamViewer() {
	// TODO Auto-generated destructor stub
}

int MultiCamViewer::run_viewer(void) {

	setup_scene();

	viewer.setUpViewInWindow(100, 100, win_width, win_height, 0);
	set_window_title(&viewer, "MultiCamViewer:  " + _dataset_path, 80, 80);
	viewer.setSceneData(scene_root);

	//Set the home position of the camera in the free viewpoint viewer:
	//-----------
	osg::ref_ptr<osgGA::TrackballManipulator> tb =
			new osgGA::TrackballManipulator;
	osg::Vec3 eye(0.75f, -1.2f, -3.0f);
	osg::Vec3 centre(0.f, 0.f, 1.5f);
	osg::Vec3 up(0.f, -1.f, 0.f);
	tb->setHomePosition(eye, centre, up, false);
	viewer.setCameraManipulator(tb);
	//-----------

	//Add event handler for keyboard control:
	viewer.addEventHandler(this);
	//Show framerate when [s] key is pressed (and other stats with subsequent presses)
	viewer.addEventHandler(new osgViewer::StatsHandler);

	update_dynamics();

	//Set background colour:
	viewer.getCamera()->setClearColor(osg::Vec4(0.88f, 0.88f, 0.90f, 1.0f));

	//osg::ref_ptr<EventHandlingClass> ctrler =
	//		new EventHandlingClass(&path, &file_names, root, &frame_geom);
	//viewer.addEventHandler(ctrler.get());

	//Free viewpoint rendering:
	rgb_render_interactive_view->allocateImage(win_width, win_height, 1,
	GL_RGBA, GL_UNSIGNED_BYTE);
	viewer.getCamera()->attach(osg::Camera::COLOR_BUFFER,
			rgb_render_interactive_view.get());

	return viewer.run();
}

void MultiCamViewer::setup_scene(void) {
	//Global axes:
	//---------------------------
	osg::ref_ptr<osg::Geode> axes = create_axes();
	scene_root->addChild(axes);
	//---------------------------

	//Frame number text:
	//---------------------------
	osg::ref_ptr<osg::Geode> text_geode = new osg::Geode;
	//Very NB! This next line stops the program hanging when I change the text value:
	frame_num_text->setDataVariance(DYNAMIC);
	text_geode->addDrawable(frame_num_text);
	osg::ref_ptr<osg::Camera> text_cam = create_hud_camera(0, win_width, 0,
			win_height);
	text_cam->addChild(text_geode.get());
	scene_root->addChild(text_cam);
	//---------------------------

	//Add the geometry for each camera to to the scene:
	//---------------------------
	for (unsigned int i = 0; i < camera_arr.size(); i++) {
		cam_vis_switch->addChild(camera_arr[i]->cam_group, true);
	}
	scene_root->addChild(cam_vis_switch);
	//---------------------------

	//Switch for each view of the skeleton

	scene_root->addChild(skel_vis_switch.get());
	scene_root->addChild(skel_fitting_switch.get());
}

void MultiCamViewer::set_window_title(osgViewer::Viewer* viewer,
		std::string win_name, int x, int y) {
	//Set the title-bar of the interactive viewer:
	//(http://weetbix.wordpress.com/2009/06/27/open-scene-graph-changing-window-title/)
	//-----------
	//Get the traits of the current window
	osg::ref_ptr<osg::GraphicsContext::Traits> traits =
			new osg::GraphicsContext::Traits(
					*viewer->getCamera()->getGraphicsContext()->getTraits());
	//Set the title
	traits->windowName = win_name;
	traits->x = x;
	traits->y = y;

	//Create a new graphics context with the modified traits
	osg::ref_ptr<osg::GraphicsContext> gc =
			osg::GraphicsContext::createGraphicsContext(traits.get());

	//Create the new camera which is a copy of the current camera in the viewer
	osg::ref_ptr<osg::Camera> cam = new osg::Camera(*viewer->getCamera());
	//Set the cameras graphics context to the gc we made above
	cam->setGraphicsContext(gc);

	//Assign the viewer the new camera
	viewer->setCamera(cam.get());
	//-----------
}

bool MultiCamViewer::handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa) {
	//Show the visualizations from the last rendered frame:
	//display_raw_image_overlay();//TEMP - disable the overlay display for speed

	switch (ea.getEventType()) {
	case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {

//		//Extract the whole core scene to a single OBJ file:
//		case osgGA::GUIEventAdapter::KEY_E:
//			//DEBUG:
//			//std::cout << "[e] pressed" << std::endl;
//			extract_core_scene_to_ply();
//			break;
//

		//Toggle aplha of colour rendering:
		case osgGA::GUIEventAdapter::KEY_A:
			if (alpha == 0.f)
				alpha = 0.7f;
			else {
				alpha = 0.f;
			}
			update_dynamics();
			break;

			//Toggle colour rendering:
		case osgGA::GUIEventAdapter::KEY_C:
			with_colour = !with_colour;
			update_dynamics();
			break;

			//Toggle cam 1 visibility:
		case osgGA::GUIEventAdapter::KEY_1:
			cam_vis_switch->setValue(0, !cam_vis_switch->getValue(0));
			update_dynamics();
			break;

			//Toggle cam 2 visibility:
		case osgGA::GUIEventAdapter::KEY_2:
			cam_vis_switch->setValue(1, !cam_vis_switch->getValue(1));
			update_dynamics();
			break;

			//Toggle cam 3 visibility:
		case osgGA::GUIEventAdapter::KEY_3:
			cam_vis_switch->setValue(2, !cam_vis_switch->getValue(2));
			update_dynamics();
			break;

			//Toggle all cameras visibility
		case osgGA::GUIEventAdapter::KEY_4:
			cam_vis_switch->setValue(0, !cam_vis_switch->getValue(0));
			cam_vis_switch->setValue(1, !cam_vis_switch->getValue(1));
			cam_vis_switch->setValue(2, !cam_vis_switch->getValue(2));
			update_dynamics();
			break;

			//Toggle skel cam 1 visibility:
		case osgGA::GUIEventAdapter::KEY_Q:
			skel_vis_switch->setValue(0, !skel_vis_switch->getValue(0));
			update_dynamics();
			break;

			//Toggle skel cam 2 visibility:
		case osgGA::GUIEventAdapter::KEY_W:
			skel_vis_switch->setValue(1, !skel_vis_switch->getValue(1));
			update_dynamics();
			break;

			//Toggle skel cam 3 visibility:
		case osgGA::GUIEventAdapter::KEY_E:
			skel_vis_switch->setValue(2, !skel_vis_switch->getValue(2));
			update_dynamics();
			break;

			//Toggle all skel cam visibility:
		case osgGA::GUIEventAdapter::KEY_R:
			skel_vis_switch->setValue(0, !skel_vis_switch->getValue(0));
			skel_vis_switch->setValue(1, !skel_vis_switch->getValue(1));
			skel_vis_switch->setValue(2, !skel_vis_switch->getValue(2));
			update_dynamics();
			break;

			//Toggle merged skeleton visibility:
		case osgGA::GUIEventAdapter::KEY_T:
			skel_vis_switch->setValue(3, !skel_vis_switch->getValue(3));
			update_dynamics();
			break;

			//Write out the entire rendered sequence:
		case osgGA::GUIEventAdapter::KEY_Y:
			//DEBUG:
			//std::cout << "[y] pressed" << std::endl;

			//Pause:
			paused = true;
			//Cycle through frames and save them:
			for (int i = begin_frame_no; i <= end_frame_no; i++) {
				disp_frame_no = i;
				update_dynamics();
				viewer.frame();
				viewer.frame(); // necessary to avoid incomplete rendering in buffers
				save_image_freeview();
			}
			break;

			//Load skeleton from a file:
		case osgGA::GUIEventAdapter::KEY_L:
			skel_controller.load_skeleton_from_file("joint_points.txt");
			break;

			//Save skeleton to file:
		case osgGA::GUIEventAdapter::KEY_K:
			skel_controller.save_skeleton_to_file("joint_points.txt");
			break;

			//Exit the app, this avoids the clean up errors.
		case osgGA::GUIEventAdapter::KEY_Escape:
			exit( EXIT_SUCCESS);
			break;

			//Toggle playing/pausing
		case osgGA::GUIEventAdapter::KEY_P:
			//DEBUG:
			//std::cout << "[p] pressed" << std::endl;
			paused = (!paused);
			break;

			//Increment frame number
		case osgGA::GUIEventAdapter::KEY_Right:
			//DEBUG:
			//std::cout << "[Right] pressed" << std::endl;
			if (disp_frame_no < end_frame_no) {
				disp_frame_no++;
				update_dynamics();
			} else {
				update_dynamics();
			}
			break;

			//Decrement frame number:
		case osgGA::GUIEventAdapter::KEY_Left:
			//DEBUG:
			//std::cout << "[Left] pressed" << std::endl;
			if (disp_frame_no > begin_frame_no) {
				disp_frame_no--;
				update_dynamics();
			} else {
				update_dynamics();
			}
			break;

		default:
			break;
		}
		break;
	default:
		break;
	}

	//Play the sequence:
	if (!paused) {
		if (disp_frame_no < end_frame_no) {

			double current_ticks = (double) cv::getTickCount();
			float elapsed_time = (current_ticks - last_frame_tick_count)
					/ ((double) cv::getTickFrequency());
			if (elapsed_time > frame_period_s) {

				//Set last ticks:
				last_frame_tick_count = current_ticks;

				//Increment the frame:
				disp_frame_no++;
				update_dynamics();
			}
		}
		//Reset to beginning:
		else {
			disp_frame_no = begin_frame_no;
			//Don't automatically replay:
			paused = true;
		}
	}

	skel_controller.handle(ea, aa);
	return false;
}

void MultiCamViewer::update_dynamics(void) {
	//Update the text displaying the frame number:
	//------------------------------------------
	char text[1024];
	sprintf(text, "Frame range %d - %d, displaying frame: %d", begin_frame_no,
			end_frame_no, disp_frame_no);
	frame_num_text->setText(std::string(text));
	//------------------------------------------

	//Update the depth map geometry for all cameras:
	//------------------------------------------
	for (unsigned int i = 0; i < camera_arr.size(); i++) {
		//DEBUG:
		//cout << "Updating depth map polygonisation for " << camera_arr[i]->get_cam_name() <<
		//		" with frame " << disp_frame_no << endl;
		const cv::Mat* depth = (camera_arr[i]->get_depth_map(disp_frame_no));
		cv::Mat* rgb = (camera_arr[i]->get_rgb_image(disp_frame_no));
		//camera_arr[i]->depth_poly.polygonise_depth_map(
		//		depth, rgb, camera_arr[i]->get_K_rgb(), camera_arr[i]->get_vis_colour(),
		//		with_colour, alpha);
		camera_arr[i]->depth_surf.surfelise_depth_map(depth, rgb,
				camera_arr[i]->get_K_f3x3(), camera_arr[i]->get_inv_K_f3x3(),
				camera_arr[i]->get_vis_colour(), with_colour, alpha);

		//Visualize the path detections in 3D:
		//------------------------------------------------------

		//(these are now pre-computed in the camera constructor)
		std::vector<std::vector<float3> >& surface_paths_3d =
				camera_arr[i]->frame_surface_paths_3d[disp_frame_no];
		//camera_arr[i]->get_surface_paths_3d(disp_frame_no, surface_paths_3d);

		//Timer t_3d_vis("3d_vis");

		camera_arr[i]->skel_vis_group->removeChildren(0,
				camera_arr[i]->skel_vis_group->getNumChildren());

		for (unsigned int sp = 0; sp < surface_paths_3d.size(); sp++) {
			osg::ref_ptr<osg::Geode> path_vis_geode = new osg::Geode;
			osg::ref_ptr<osg::Geometry> path_vis_geom = new osg::Geometry;
			osg::ref_ptr<osg::Vec3Array> path_vis_vertices = new osg::Vec3Array(
					surface_paths_3d[sp].size());
			osg::ref_ptr<osg::DrawElementsUInt> path_vis_indices =
					new osg::DrawElementsUInt(GL_LINES,
							2 * surface_paths_3d[sp].size());

			for (unsigned int element = 0;
					element < surface_paths_3d[sp].size(); element++) {
				(*path_vis_vertices)[element].set(
						osg::Vec3(surface_paths_3d[sp][element].x,
								surface_paths_3d[sp][element].y,
								surface_paths_3d[sp][element].z));
			}
			for (unsigned int element = 0;
					element < surface_paths_3d[sp].size() - 1; element++) {
				(*path_vis_indices)[2 * element] = element;
				(*path_vis_indices)[2 * element + 1] = element + 1;
			}

			path_vis_geom->setVertexArray(path_vis_vertices.get());
			path_vis_geom->addPrimitiveSet(path_vis_indices.get());
			path_vis_geode->addDrawable(path_vis_geom.get());

			//Wireframe with lighting turned off:
			osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode;
			pm->setMode(osg::PolygonMode::FRONT_AND_BACK,
					osg::PolygonMode::LINE);
			path_vis_geode->getOrCreateStateSet()->setAttribute(pm.get());
			path_vis_geode->getOrCreateStateSet()->setMode( GL_LIGHTING,
					osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
			//Set colour:
			osg::ref_ptr<osg::Vec4Array> colours = new osg::Vec4Array;
			colours->push_back(
					osg::Vec4(hsv_2_rgb(osg::Vec3(360 * (sp / 11.f), 0.6, 0.7)),
							1));
			path_vis_geom->setColorArray(colours.get());
			path_vis_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			//Make the lines thicker:
			//-------------
			osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth();
			linewidth->setWidth(2.0f);
			path_vis_geode->getOrCreateStateSet()->setAttributeAndModes(
					linewidth, osg::StateAttribute::ON);
			//-------------

			camera_arr[i]->skel_vis_group->addChild(path_vis_geode);

			//Add a sphere at the first element:
			//--------------------------------------
			osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable;
			sphere->setShape(new osg::Sphere(path_vis_vertices->at(0), 0.02f));
			//sphere->setColor( colours->at(0) );
			sphere->setColor(osg::Vec4(1, 1, 1, 1));
			osg::ref_ptr<osg::Geode> sphere_geode = new osg::Geode;
			sphere_geode->addDrawable(sphere.get());
			camera_arr[i]->skel_vis_group->addChild(sphere_geode);
			//--------------------------------------

//			//Add a sphere at the all element until the second is 10cm (Euclidean) from the base:
//			//--------------------------------------
//			for(int element = 0; element < surface_paths_3d[sp].size(); element++)
//			{
//				if((path_vis_vertices->at(element) - path_vis_vertices->at(0)).length() < 0.10)
//				{
//					osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable;
//					sphere->setShape( new osg::Sphere(path_vis_vertices->at(element),
//							0.02f) );
//					//sphere->setColor( colours->at(0) );
//					sphere->setColor(osg::Vec4(1,1,1,1));
//					osg::ref_ptr<osg::Geode> sphere_geode = new osg::Geode;
//					sphere_geode->addDrawable(sphere.get());
//					camera_arr[i]->skel_vis_group->addChild(sphere_geode);
//				}
//				else
//				{
//					break;
//				}
//			}
//			//--------------------------------------

		}

		//------------------------------------------------------

		//t_3d_vis.tock_print();

	}
	//skel_renderer.update_dynamics(disp_frame_no);
	skel_controller.update_dynamics(disp_frame_no);
	//------------------------------------------

//	//DEBUG TEST: Render POV
//	//------------------------------------------
//	if(disp_frame_no == 111)
//	{
//		osg::Matrix T_relative = cvmat4x4_2_osgmat(camera_arr[1]->get_T_rgb());
//		osg::Matrix3 K = cvmat3x3_2_osgmat(camera_arr[1]->get_K_rgb());
//		cv::Mat result(480,640,CV_16U);
//
//		RenderPOV rp(640,480);
//		rp.render_pov(camera_arr[1]->depth_surf.depth_surfel_geode, &result, K, T_relative);
//
//		cv::imshow("Render POV result", result*16);
//
//		cv::imshow("Original frame", *(camera_arr[1]->get_depth_map(disp_frame_no))*16);
//
//		cv::Mat float_render;
//		cv::Mat float_orig;
//		result.convertTo(float_render, CV_32F);
//		(camera_arr[1]->get_depth_map(disp_frame_no))->convertTo(float_orig, CV_32F);
//		cv::Mat diff = cv::abs(float_render - float_orig);
//		cv::imshow("Diff", diff/10);
//
//		//Save the diff:
//		//cv::Mat diff_8bit;
//		//diff.convertTo(diff_8bit, CV_8U, 25.0);
//		//cv::imwrite("test_diff.png", diff_8bit);
//
//		cv::waitKey(60);
//		paused = true;
//	}
//	//------------------------------------------

}

osgText::Text* MultiCamViewer::create_text(const osg::Vec3& pos,
		const std::string& content, float size) {
	osg::ref_ptr<osgText::Font> g_font = osgText::readFontFile("FreeSans.ttf");

	osg::ref_ptr<osgText::Text> text = new osgText::Text;
	text->setFont(g_font.get());
	text->setCharacterSize(size);
	text->setAxisAlignment(osgText::TextBase::XY_PLANE);
	text->setPosition(pos);
	text->setColor(osg::Vec4(0.2, 1.0, 0.2, 1));
	text->setText(content);
	return text.release();
}

osg::Camera* MultiCamViewer::create_hud_camera(double left, double right,
		double bottom, double top) {
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setClearMask( GL_DEPTH_BUFFER_BIT);
	camera->setRenderOrder(osg::Camera::POST_RENDER);
	camera->setAllowEventFocus(false);
	camera->setProjectionMatrix(osg::Matrix::ortho2D(left, right, bottom, top));
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING,
			osg::StateAttribute::OFF);
	return camera.release();
}

void MultiCamViewer::save_image_freeview() {
	char fn[1024];
	sprintf(fn, "%s/Desktop/Viewer_Rendering/%05d.png", getenv("HOME"),
			disp_frame_no);
	std::string rgb_freeview_fn(fn);

	//Save RGB image:
	if (!osgDB::writeImageFile(*rgb_render_interactive_view, rgb_freeview_fn)) {
		std::cerr << "Failed to save: " << rgb_freeview_fn << endl;
		exit(-1);
	}

}

/*
 * MultiCamViewer.cpp
 *
 *  Created on: 9 Jul 2013
 *      Author: cm00215
 */

#include "MultiCamViewer.h"

using std::cout;
using std::endl;

extern bool MiscUtils::use_normal_shader;
extern float3x3 Projections::K;
extern float3x3 Projections::invK;

MultiCamViewer::MultiCamViewer(std::string path) :
		win_width(1280), win_height(720), paused(true), with_colour(false), frame_period_s(
				1.0 / 30.0), //30fps
		last_frame_tick_count(0), manual_origin_set(false), manual_axis_rot(
				false), set_ground_truth(true), show_bounding_box(false), current_axis_manual(
				0), last_cam_index(0), _dataset_path(path), scene_root(
				new osg::Group()), rgb_render_interactive_view(new osg::Image), cam_vis_switch(
				new osg::Switch), render_skel_group(new osg::Group()), ground_truth_group(
				new osg::Group()), frame_num_text(
				create_text(osg::Vec3(20.0f, 20.0f, 0.0f),
						"Frame range XXX-XXX, displaying frame: XXX", 18.0f)), alpha(
				0.f), cam_calibrator(camera_arr), num_user_points(0), max_user_points(
				16), skel_controller(camera_arr, render_skel_group), ground_truth(
				max_user_points + 4) {

	user_points.resize(max_user_points);
	//When manual origin set use camera colour, if not then user normals
	//for the shader
	MiscUtils::use_normal_shader = !manual_origin_set;

	//Get the list of cameras and construct camera objects for them:
	std::vector<std::string> cam_names;
	MiscUtils::get_dir_names(path, &cam_names);
	for (unsigned int i = 0; i < cam_names.size(); i++) {
		RGBD_CameraPtr cam(new RGBD_Camera(_dataset_path, cam_names[i]));
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

	Projections::K = camera_arr.front()->get_K_f3x3();
	Projections::invK = camera_arr.front()->get_inv_K_f3x3();

	//TODO This should be given by the user or somehow calculated from
	//the images
	//Set bounding box limits
	bounding_box.set(-1.3, -0.54, -0.25, 1.3, -0.015, 0.25);

	if (!manual_origin_set) {
		//Remove background using bounding box
		for (unsigned int i = 0; i < camera_arr.size(); i++) {
			camera_arr[i]->remove_background_only_bounding_box(bounding_box);
		}
		skel_controller.generate_skeletonization();
	}

	//DEBUG:
	//cout << "begin_frame_no: " << begin_frame_no << " end_frame_no: " << end_frame_no << endl;

	//Set currently displayed frame to beginning:
	disp_frame_no = begin_frame_no;
}

MultiCamViewer::~MultiCamViewer() {
}

int MultiCamViewer::run_viewer() {

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

void MultiCamViewer::setup_scene() {
	//Global axis:
	//---------------------------
	osg::ref_ptr<osg::Geode> axis;
	if (!manual_axis_rot) {
		axis = MiscUtils::create_axis();
	} else {
		axis = MiscUtils::create_axis(100);
	}
	scene_root->addChild(axis);
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

	scene_root->addChild(render_skel_group);

	if (!manual_origin_set) {
		skel_controller.setup_scene();
	}

	if (show_bounding_box) {
		//Shows bounding box used to removed the background
		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		osg::Vec3 lengths(bounding_box.xMax() - bounding_box.xMin(),
				bounding_box.yMax() - bounding_box.yMin(),
				bounding_box.zMax() - bounding_box.zMin());
		geode->addDrawable(
				new osg::ShapeDrawable(
						new osg::Box(bounding_box.center(), lengths.x(),
								lengths.y(), lengths.z())));
		osg::StateSet* ss = geode->getOrCreateStateSet();
		ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		ss->setAttributeAndModes(
				new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,
						osg::PolygonMode::LINE));
		scene_root->addChild(geode);
	}

	scene_root->addChild(ground_truth_group);

	if (ground_truth.is_data_loaded()) {
		int index_offset = 0;
		for (unsigned int i = 0; i < max_user_points; i++) {
			display_sphere(
					ground_truth.get_point(i + index_offset, disp_frame_no),
					ground_truth_group, i);

			if (i == 1 || i == 4 || i == 8 || i == 11) {
				index_offset++;
			}
		}
	}
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
			last_cam_index = 0;
			update_dynamics();
			break;

			//Toggle cam 2 visibility:
		case osgGA::GUIEventAdapter::KEY_2:
			cam_vis_switch->setValue(1, !cam_vis_switch->getValue(1));
			last_cam_index = 1;
			update_dynamics();
			break;

			//Toggle cam 3 visibility:
		case osgGA::GUIEventAdapter::KEY_3:
			cam_vis_switch->setValue(2, !cam_vis_switch->getValue(2));
			last_cam_index = 3;
			update_dynamics();
			break;

			//Toggle all cameras visibility
		case osgGA::GUIEventAdapter::KEY_4:
			cam_vis_switch->setValue(0, !cam_vis_switch->getValue(0));
			cam_vis_switch->setValue(1, !cam_vis_switch->getValue(1));
			cam_vis_switch->setValue(2, !cam_vis_switch->getValue(2));
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

			//Exit the app, this avoids the clean up errors.
		case osgGA::GUIEventAdapter::KEY_Escape:
			exit(EXIT_SUCCESS);
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

		case osgGA::GUIEventAdapter::KEY_V:
			if (manual_origin_set && manual_axis_rot) {
				cam_calibrator.manual_axis_rotation(0.001, current_axis_manual);
				update_dynamics();
			}
			break;
		case osgGA::GUIEventAdapter::KEY_B:
			if (manual_origin_set && manual_axis_rot) {
				cam_calibrator.manual_axis_rotation(-0.001,
						current_axis_manual);
				update_dynamics();
			}
			break;
		case osgGA::GUIEventAdapter::KEY_N:
			if (manual_origin_set && manual_axis_rot) {
				current_axis_manual++;
				if (current_axis_manual == 3) {
					current_axis_manual = 0;
				}
			}
			break;
		case osgGA::GUIEventAdapter::KEY_M:
			if (manual_origin_set && manual_axis_rot) {
				cam_calibrator.save_all_cameras(_dataset_path);
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

	if (manual_origin_set
			&& ea.getEventType() == osgGA::GUIEventAdapter::RELEASE
			&& ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON
			&& (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {
		set_calibration_point(ea, aa);
	}

	if (set_ground_truth && ea.getEventType() == osgGA::GUIEventAdapter::RELEASE
			&& ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON
			&& (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {
		set_ground_truth_point(ea, aa);
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

	return skel_controller.handle(ea, aa);
}

void MultiCamViewer::update_dynamics() {
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
			path_vis_geode->getOrCreateStateSet()->setMode(GL_LIGHTING,
					osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
			//Set colour:
			osg::ref_ptr<osg::Vec4Array> colours = new osg::Vec4Array;
			colours->push_back(
					osg::Vec4(
							MiscUtils::hsv_2_rgb(
									osg::Vec3(360 * (sp / 11.f), 0.6, 0.7)),
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

	if (!manual_origin_set) {
		skel_controller.update_dynamics(disp_frame_no);
	}

	if (set_ground_truth) {
		int index_offset = 0;
		for (unsigned int i = 0; i < ground_truth_group->getNumChildren();
				i++) {
			display_sphere(
					ground_truth.get_point(i + index_offset, disp_frame_no),
					ground_truth_group, i);

			if (i == 1 || i == 4 || i == 8 || i == 11) {
				index_offset++;
			}
		}
	}

	num_user_points = 0;
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
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
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

void MultiCamViewer::set_calibration_point(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa) {

	osg::Vec3 pos;
	if (get_user_point(ea, aa, pos)) {
		if (num_user_points < 4) {
			user_points[num_user_points] = pos;
			num_user_points++;

			//Draw a sphere to give user feedback
			osg::ref_ptr<osg::Geode> sphere_geode = new osg::Geode;
			osg::ref_ptr<osg::ShapeDrawable> sphere_shape;
			sphere_shape = new osg::ShapeDrawable(new osg::Sphere(pos, 0.01));
			sphere_shape->setColor(osg::Vec4(1.0, 0.0, 0.0, 0.0));

			sphere_geode->getOrCreateStateSet()->setMode(GL_LIGHTING,
					osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

			sphere_geode->addDrawable(sphere_shape);
			scene_root->addChild(sphere_geode.get());

		} else {
			cam_calibrator.set_plate_points(user_points[0], user_points[1],
					user_points[2], user_points[3]);
			//cam_cal.recalibrate_center_all_cameras();
			cam_calibrator.recalibrate_axis_camera();
			cam_calibrator.save_all_cameras(_dataset_path);

			//Code to calibrate each camera axis separately
			//cam_cal.recalibrate_axis_camera();
			//cam_cal.save_camera_calibration(last_cam_index, _dataset_path);
		}
	}
}

void MultiCamViewer::set_ground_truth_point(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa) {
	osg::Vec3 pos;
	if (get_user_point(ea, aa, pos)) {
		if (num_user_points < max_user_points) {
			user_points[num_user_points] = pos;
			display_sphere(pos, ground_truth_group, num_user_points);
			num_user_points++;
		} else {
			ground_truth.set_ground_truth_frame(user_points, disp_frame_no);

			//std::string path(
			//		"/home/cvssp/misc/m04701/workspace/data/groundTruth/test.txt");
			//ground_truth.save_data(path);
			num_user_points = 0;
		}
	}
}

bool MultiCamViewer::get_user_point(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa, osg::Vec3& point) {

	osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
	if (viewer) {
		double SELECTION_SENSITIVITY = 2;
		osg::ref_ptr<osg::Viewport> viewport =
				viewer->getCamera()->getViewport();
		double mx = viewport->x()
				+ (int) ((double) viewport->width()
						* (ea.getXnormalized() * 0.5 + 0.5));
		double my = viewport->y()
				+ (int) ((double) viewport->height()
						* (ea.getYnormalized() * 0.5 + 0.5));
		double w = SELECTION_SENSITIVITY;
		double h = SELECTION_SENSITIVITY;
		//Since we want to select polygons we need the polytope intersector
		osg::ref_ptr<osgUtil::PolytopeIntersector> intersector =
				new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
						mx - w, my - h, mx + w, my + h);

		osgUtil::IntersectionVisitor iv(intersector.get());
		iv.setTraversalMask(~0x1);
		viewer->getCamera()->accept(iv);

		if (intersector->containsIntersections()) {
			std::multiset<osgUtil::PolytopeIntersector::Intersection>::iterator result =
					intersector->getIntersections().begin();

			osg::MatrixList worldMatrices =
					result->drawable->getWorldMatrices();

			osg::MatrixList::iterator itr = worldMatrices.begin();
			osg::Matrix& matrix = *itr;
			//Get global coordinates of the picked point
			point = result->localIntersectionPoint * matrix;
			return true;
		}
	}
	return false;
}

void MultiCamViewer::display_sphere(const osg::Vec3& pos,
		osg::Group* add_to_group, unsigned int num_sphere) {
	if (num_sphere < add_to_group->getNumChildren()) {
		osg::Geode* sphe_geo = add_to_group->getChild(num_sphere)->asGeode();
		osg::ShapeDrawable* sphe_shape =
				static_cast<osg::ShapeDrawable*>(sphe_geo->getDrawable(0));
		osg::Sphere* sphere = static_cast<osg::Sphere*>(sphe_shape->getShape());
		sphere->setCenter(pos);
		sphe_shape->dirtyBound();
	} else {
		//Draw a sphere to give user feedback
		osg::ref_ptr<osg::Geode> sphere_geode = new osg::Geode;
		osg::ref_ptr<osg::ShapeDrawable> sphere_shape;
		sphere_shape = new osg::ShapeDrawable(new osg::Sphere(pos, 0.01));
		sphere_shape->setColor(osg::Vec4(1.0, 0.0, 0.0, 0.0));

		sphere_geode->getOrCreateStateSet()->setMode(GL_LIGHTING,
				osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

		sphere_geode->addDrawable(sphere_shape);

		sphere_shape->setUseDisplayList(false);
		sphere_shape->setUseVertexBufferObjects(true);

		add_to_group->addChild(sphere_geode.get());
	}
}

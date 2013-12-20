CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

SET( PROJECT_NAME MultiCamViewer ) 
PROJECT( ${PROJECT_NAME} ) 
SET( PROJECT_EXE "${PROJECT_NAME}_exe" ) 

#Boost
FIND_PACKAGE( Boost 1.46 REQUIRED COMPONENTS filesystem system )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

INCLUDE(FindFreetype)
MESSAGE(STATUS "Freetype lib dir found at ${FREETYPE_LIBRARIES}")
MESSAGE(STATUS "Freetype include dir found at ${FREETYPE_INCLUDE_DIRS}")

#Custom path to compiled by hand libraries, thanks to not having permision to install software
#If you have installation permisions remove this line
SET( CMAKE_PREFIX_PATH "/home/cvssp/misc/m04701/customlibs" )

#OpenCV
FIND_PACKAGE( OpenCV REQUIRED ) 
MESSAGE(STATUS "OpenCV include dir found at ${OpenCV_INCLUDE_DIRS}") 
MESSAGE(STATUS "OpenCV lib dir found at ${OpenCV_LIB_DIR}")

#OSG
find_package(OpenThreads )
find_package(osg)
find_package(osgDB )
find_package(osgUtil )
find_package(osgViewer )
find_package(osgGA )
find_package(osgText )
find_package(osgAnimation )

#CUDA
FIND_PACKAGE( CUDA )
INCLUDE_DIRECTORIES( ${CUDA_INCLUDE_DIRS} )
MESSAGE(STATUS "Cuda lib dir found at ${CUDA_INCLUDE_DIRS}")

#Source and header files: 
SET(SOURCE 
src/BVHFormat.cpp
src/DepthMapPoly.cpp
src/DepthMapSurfel.cpp
src/Extras.cpp
src/main.cpp
src/MessageHandler.cpp
src/MocapData.cpp
src/MocapHeader.cpp
src/MultiCamViewer.cpp
src/Node.cpp
src/PointFusion.cpp
src/RenderPOV.cpp
src/RenderSkeletonization.cpp
src/RGBDCamera.cpp
src/RGBDFrame.cpp
src/SimpleTimer.cpp
src/SkeletonController.cpp
src/Skeleton.cpp
src/SkeletonFitting.cpp
src/Skeletonization2D.cpp
src/Skeletonization3D.cpp
src/SkeletonMixer.cpp
src/SkeletonState.cpp
src/SurfelModel.cpp
src/IKAN/myvec.cxx
src/IKAN/myvec.cxx
src/IKAN/srs.cxx
src/IKAN/trig.cxx
) 

SET(HEADERS 
src/BVHFormat.h
src/CudaVec.h
src/DebugUtil.h
src/DepthMapPoly.h
src/DepthMapSurfel.h
src/Extras.h
src/MessageHandler.h
src/MiscUtils.h
src/MocapData.h
src/MocapHeader.h
src/MultiCamViewer.h
src/Node.h
src/PointFusion.h
src/RenderPOV.h
src/RenderSkeletonization.h
src/RGBDCamera.h
src/RGBDFrame.h
src/SimpleTimer.h
src/SkeletonController.h
src/SkeletonFitting.h
src/Skeleton.h
src/Skeletonization2D.h
src/Skeletonization3D.h
src/SkeletonMixer.h
src/SkeletonState.h
src/SurfelModel.h
src/IKAN/myvec.h
src/IKAN/srs.h
src/IKAN/trig.h
) 


macro( config_project PROJNAME LIBNAME )
	include_directories( ${${LIBNAME}_INCLUDE_DIR} )
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

#Create the executable
ADD_EXECUTABLE( ${PROJECT_EXE} ${SOURCE} ${HEADERS} ) 

#OSG linking
config_project( ${PROJECT_EXE} OPENTHREADS )
config_project( ${PROJECT_EXE} OSG )
config_project( ${PROJECT_EXE} OSGDB )
config_project( ${PROJECT_EXE} OSGUTIL )
config_project( ${PROJECT_EXE} OSGVIEWER )
config_project( ${PROJECT_EXE} OSGGA )
config_project( ${PROJECT_EXE} OSGSIM )
config_project( ${PROJECT_EXE} OSGWIDGET )
config_project( ${PROJECT_EXE} OSGANIMATION )
config_project( ${PROJECT_EXE} OSGTEXT )

#OpenCV linking
target_link_libraries( ${PROJECT_EXE} ${OpenCV_LIBS} )

#Boost linking
target_link_libraries( ${PROJECT_EXE} ${Boost_LIBRARIES} )

INCLUDE_DIRECTORIES(${FREETYPE_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(${PROJECT_EXE} ${FREETYPE_LIBRARIES})

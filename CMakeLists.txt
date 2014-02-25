CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

SET(PROJECT_NAME MultiCamViewer) 
PROJECT(${PROJECT_NAME}) 
SET(PROJECT_EXE "${PROJECT_NAME}_exe") 

#Set build to Release is none is given
if (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
endif()    

#Add custom .cmake files located in this folder
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmakeModules")

#Boost
FIND_PACKAGE(Boost 1.46 REQUIRED COMPONENTS filesystem system)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

#CUDA
SET(CUDA_INCLUDE_DIRS "dependencies/CUDA")
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

#OpenCV
FIND_PACKAGE(OpenCV REQUIRED HINTS "/home/cvssp/misc/m04701/customlibs") 
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
MESSAGE(STATUS "OpenCV include dir found at ${OpenCV_INCLUDE_DIRS}") 
MESSAGE(STATUS "OpenCV lib dir found at ${OpenCV_LIB_DIR}")

#Custom path to compiled by hand libraries, thanks to not having permision to install software
#If you have installation permisions remove this line
SET(CMAKE_PREFIX_PATH "/home/cvssp/misc/m04701/customlibs")

#FLANN
FIND_PACKAGE(Flann)
INCLUDE_DIRECTORIES(${FLANN_INCLUDE_DIRS})

#KDL
FIND_PACKAGE(KDL)
MESSAGE(STATUS "KDL DIR ${KDL_INCLUDE_DIRS}")
MESSAGE(STATUS "KDL LIB ${KDL_LIBS}")
INCLUDE_DIRECTORIES( ${KDL_INCLUDE_DIRS} )

#TODO HINT does not work with osg, not sure why
#OSG
FIND_PACKAGE(OpenThreads)
FIND_PACKAGE(osg)
FIND_PACKAGE(osgDB)
FIND_PACKAGE(osgUtil)
FIND_PACKAGE(osgViewer)
FIND_PACKAGE(osgGA)
FIND_PACKAGE(osgText)
FIND_PACKAGE(osgAnimation)

#Source and header files: 
SET(SOURCE
src/BodyFitter.cpp
src/BonePosFinder.cpp
src/BVHFormat.cpp
src/CameraCalibrator.cpp
src/CloudClusterer.cpp
src/CommonFitter.cpp
src/DepthMapPoly.cpp
src/DepthMapSurfel.cpp
src/IKSolver.cpp
src/KNNSearch.cpp
src/LegFitter.cpp
src/main.cpp
src/MessageHandler.cpp
src/MiscUtils.cpp
src/MocapData.cpp
src/MocapHeader.cpp
src/MultiCamViewer.cpp
src/Node.cpp
src/PixelSearch.cpp
src/PointFusion.cpp
src/Projections.cpp
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
) 

SET(HEADERS
src/BodyFitter.h
src/BonePosFinder.h
src/BVHFormat.h
src/CameraCalibrator.h
src/CloudClusterer.h
src/CommonFitter.h
src/CompMethods.h
src/CudaVec.h
src/DebugUtil.h
src/DepthMapPoly.h
src/DepthMapSurfel.h
src/IKSolver.h
src/KNNSearch.h
src/LegFitter.h
src/MessageHandler.h
src/MiscUtils.h
src/MocapData.h
src/MocapHeader.h
src/MultiCamViewer.h
src/Node.h
src/PixelSearch.h
src/PointFusion.h
src/Projections.h
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
) 


MACRO(config_project PROJNAME LIBNAME)
	INCLUDE_DIRECTORIES(${${LIBNAME}_INCLUDE_DIR})
	TARGET_LINK_LIBRARIES(${PROJNAME} ${${LIBNAME}_LIBRARY})
ENDMACRO()

#Create the executable
ADD_EXECUTABLE(${PROJECT_EXE} ${SOURCE} ${HEADERS}) 

#Boost linking
TARGET_LINK_LIBRARIES(${PROJECT_EXE} ${Boost_LIBRARIES})

#FLANN linking
TARGET_LINK_LIBRARIES(${PROJECT_EXE} ${FLANN_LIBRARIES})

#KDL linking
TARGET_LINK_LIBRARIES(${PROJECT_EXE} ${KDL_LIBS})

#OpenCV linking
TARGET_LINK_LIBRARIES(${PROJECT_EXE} ${OpenCV_LIBS})

#OSG linking
config_project(${PROJECT_EXE} OPENTHREADS)
config_project(${PROJECT_EXE} OSG)
config_project(${PROJECT_EXE} OSGDB)
config_project(${PROJECT_EXE} OSGUTIL)
config_project(${PROJECT_EXE} OSGVIEWER)
config_project(${PROJECT_EXE} OSGGA)
config_project(${PROJECT_EXE} OSGSIM)
config_project(${PROJECT_EXE} OSGWIDGET)
config_project(${PROJECT_EXE} OSGANIMATION)
config_project(${PROJECT_EXE} OSGTEXT)


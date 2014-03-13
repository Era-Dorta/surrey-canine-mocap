CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 

SET(PROJECT_NAME MultiCamViewer) 
PROJECT(${PROJECT_NAME}) 
SET(PROJECT_EXE "${PROJECT_NAME}_exe") 

set(VERBOSE "FALSE")

MACRO(PRINT LIBNAME LIBFOLDER)
	if (VERBOSE)
		MESSAGE(STATUS "Found ${LIBNAME}: ${LIBFOLDER}")
	endif()
ENDMACRO()

#Set build to Release is none is given
if (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
endif()

#Add custom .cmake files located in this folder
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmakeModules")

#Boost
FIND_PACKAGE(Boost 1.46 REQUIRED COMPONENTS filesystem system)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
PRINT("Boost include:" ${Boost_INCLUDE_DIR})

#OpenCV
FIND_PACKAGE(OpenCV REQUIRED HINTS "/home/cvssp/misc/m04701/customlibs") 
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
PRINT("OpenCV include" ${OpenCV_INCLUDE_DIRS}) 
PRINT("OpenCV lib" ${OpenCV_LIB_DIR})


#Custom path to compiled by hand libraries, thanks to not having permision to install software
#If you have installation permisions remove this line
SET(CMAKE_PREFIX_PATH "/home/cvssp/misc/m04701/customlibs")

#FLANN
FIND_PACKAGE(Flann)
INCLUDE_DIRECTORIES(${FLANN_INCLUDE_DIRS})
PRINT("Flann include" ${FLANN_INCLUDE_DIRS})


#KDL
FIND_PACKAGE(KDL)
INCLUDE_DIRECTORIES( ${KDL_INCLUDE_DIRS} )
PRINT("KDL include" ${KDL_INCLUDE_DIRS})
PRINT("KDL lib" ${KDL_LIBS})	

#PCL
FIND_PACKAGE(PCL 1.3 REQUIRED COMPONENTS common io sample_consensus filters visualization )
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

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

#Other .h files, currently CUDA
SET(DEPENDENCIES_DIRS "dependencies")
INCLUDE_DIRECTORIES(${DEPENDENCIES_DIRS})

#Source and header files: 
SET(SOURCE
src/main.cpp
src/MultiCamViewer.cpp
src/Controller/CameraCalibrator.cpp
src/Controller/GroundTruth.cpp
src/Controller/PixelSearch.cpp
src/Controller/Projections.cpp
src/Controller/SkeletonController.cpp
src/Controller/Skeletonization2D.cpp
src/Controller/Skeletonization3D.cpp
src/Controller/SkeletonMixer.cpp
src/Controller/SkeletonState.cpp
src/Controller/Fitting/BodyFitter.cpp
src/Controller/Fitting/BonePosFinder.cpp
src/Controller/Fitting/CloudClusterer.cpp
src/Controller/Fitting/CommonFitter.cpp
src/Controller/Fitting/EnhancedIKSolver.cpp
src/Controller/Fitting/IKSolver.cpp
src/Controller/Fitting/KNNSearch.cpp
src/Controller/Fitting/LegFitter.cpp
src/Controller/Fitting/SkeletonFitting.cpp
src/Misc/MessageHandler.cpp
src/Misc/MiscUtils.cpp
src/Misc/SimpleTimer.cpp
src/Model/BVHFormat.cpp
src/Model/MocapData.cpp
src/Model/MocapHeader.cpp
src/Model/Node.cpp
src/Model/Skeleton.cpp
src/View/DepthMapPoly.cpp
src/View/DepthMapSurfel.cpp
src/View/PointFusion.cpp
src/View/RenderPOV.cpp
src/View/RenderSkeletonization.cpp
src/View/RGBDCamera.cpp
src/View/RGBDFrame.cpp
src/View/SurfelModel.cpp
) 

SET(HEADERS
src/MultiCamViewer.h
src/Controller/CameraCalibrator.h
src/Controller/GroundTruth.h
src/Controller/PixelSearch.h
src/Controller/Projections.h
src/Controller/SkeletonController.h
src/Controller/Skeletonization2D.h
src/Controller/Skeletonization3D.h
src/Controller/SkeletonMixer.h
src/Controller/SkeletonState.h
src/Controller/Fitting/BodyFitter.h
src/Controller/Fitting/BonePosFinder.h
src/Controller/Fitting/CloudClusterer.h
src/Controller/Fitting/CommonFitter.h
src/Controller/Fitting/CompMethods.h
src/Controller/Fitting/EnhancedIKSolver.h
src/Controller/Fitting/IKSolver.h
src/Controller/Fitting/KNNSearch.h
src/Controller/Fitting/LegFitter.h
src/Controller/Fitting/SkeletonFitting.h
src/Misc/DebugUtil.h
src/Misc/CudaVec.h
src/Misc/MessageHandler.h
src/Misc/MiscUtils.h
src/Misc/SimpleTimer.h
src/Model/BVHFormat.h
src/Model/MocapData.h
src/Model/MocapHeader.h
src/Model/Node.h
src/Model/Skeleton.h
src/View/DepthMapPoly.h
src/View/DepthMapSurfel.h
src/View/PointFusion.h
src/View/RenderPOV.h
src/View/RenderSkeletonization.h
src/View/RGBDCamera.h
src/View/RGBDFrame.h
src/View/SurfelModel.h
) 


MACRO(CONFIG_PROJECT PROJNAME LIBNAME)
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

#PCL linking
TARGET_LINK_LIBRARIES(${PROJECT_EXE} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
	${PCL_SAMPLE_CONSESUS_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

#OSG linking
CONFIG_PROJECT(${PROJECT_EXE} OPENTHREADS)
CONFIG_PROJECT(${PROJECT_EXE} OSG)
CONFIG_PROJECT(${PROJECT_EXE} OSGDB)
CONFIG_PROJECT(${PROJECT_EXE} OSGUTIL)
CONFIG_PROJECT(${PROJECT_EXE} OSGVIEWER)
CONFIG_PROJECT(${PROJECT_EXE} OSGGA)
CONFIG_PROJECT(${PROJECT_EXE} OSGSIM)
CONFIG_PROJECT(${PROJECT_EXE} OSGWIDGET)
CONFIG_PROJECT(${PROJECT_EXE} OSGANIMATION)
CONFIG_PROJECT(${PROJECT_EXE} OSGTEXT)


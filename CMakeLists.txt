CMAKE_MINIMUM_REQUIRED(VERSION 2.8) 
#Aqui ponemos el nombre del proyecto
SET( PROJECT_NAME MultiCamViewer ) 
PROJECT( ${PROJECT_NAME} ) 
SET( PROJECT_EXE "${PROJECT_NAME}_exe" )

#OSG
find_package(OpenThreads )
find_package(osg )
find_package(osgDB )
find_package(osgUtil )
find_package(osgViewer )
find_package(osgGA )
find_package(osgText )
find_package(osgAnimation )


#OpenCV
FIND_PACKAGE( OpenCV REQUIRED ) 
MESSAGE(STATUS "OpenCV include dir found at ${OpenCV_INCLUDE_DIRS}") 
MESSAGE(STATUS "OpenCV lib dir found at ${OpenCV_LIB_DIR}")

#Boost
FIND_PACKAGE( Boost 1.46 REQUIRED COMPONENTS filesystem system )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

#CUDA
FIND_PACKAGE( CUDA )
INCLUDE_DIRECTORIES( ${CUDA_INCLUDE_DIRS} )
MESSAGE(STATUS "Cuda lib dir found at ${CUDA_INCLUDE_DIRS}")

#Source and header files: 
SET(SOURCE 
#Here go the source files
src/DepthMapPoly.cpp
src/DepthMapSurfel.cpp
src/main.cpp
src/MultiCamViewer.cpp
src/PointFusion.cpp
src/RenderPOV.cpp
src/RGBDCamera.cpp
src/RGBDFrame.cpp
src/SimpleTimer.cpp
src/SurfelModel.cpp
) 

SET(HEADERS 
#Here goes the headers files
src/CudaVec.h
src/DepthMapPoly.h
src/DepthMapSurfel.h
src/MiscUtils.h
src/MultiCamViewer.h
src/PointFusion.h
src/RenderPOV.h
src/RGBDCamera.h
src/RGBDFrame.h
src/SimpleTimer.h
src/SurfelModel.h
) 


macro( config_project PROJNAME LIBNAME )
	include_directories( ${${LIBNAME}_INCLUDE_DIR} )
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

#Make the library 
#Crea el ejecutable Project_Name usando Source y Headers
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

Canine Marker-less motion tracking
===

A simple aplication to fit and visualize virtual canine skeletons using data from Kinect cameras.

Installation
-----------

The following libraries are needed to compile the code:
* [Boost](http://www.boost.org)
* [OpenCV](http://opencv.org)
* [Flann](http://www.cs.ubc.ca/research/flann)
* [KDL](http://www.orocos.org/kdl)
* [PCL](http://pointclouds.org)
* [OSG](http://www.openscenegraph.org)

The application expects the following folders to exits in your home directory:
* `~/workspace/data/bvh/` 
* `~/workspace/data/groundTruth/`

[OpenMP](http://openmp.org/wp/) will be used for Flann if it is detected by cmake.

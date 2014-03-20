MCV
===

Markerless motion tracking for dog gait analysis.

Installation
-----------

You need the following libraries:
* [Boost](http://www.boost.org)
* [OpenCV](http://opencv.org)
* [Flann](http://www.cs.ubc.ca/research/flann)
* [KDL](http://www.orocos.org/kdl)
* [PCL](http://pointclouds.org)
* [OSG](http://www.openscenegraph.org)

You will also need the following folders in your home directory:
* `~/workspace/data/bvh/` 
* `~/workspace/data/groundTruth/`

If your compiler supports [OpenMP](http://openmp.org/wp/) the cmake compilation will automatically use it for Flann.

/*
 * PointCloud.h
 *
 *  Created on: 13 Mar 2014
 *      Author: m04701
 */

#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include "../Misc/CudaVec.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <osg/Array>
#include <opencv2/opencv.hpp>
#include <boost/make_shared.hpp>

// Interface class to hide want kind of cloud point we are using and
// be compatible with all the different vector types used in the program

class PointCloud {
public:
	PointCloud();
	PointCloud(unsigned int width, unsigned int height = 1);
	PointCloud(const osg::ref_ptr<osg::Vec3Array>& in_cloud);
	PointCloud(const std::vector<float3>& in_cloud);
	PointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud);

	void from_osgVec3Array(const osg::ref_ptr<osg::Vec3Array>& in_cloud);
	void from_float3Array(const std::vector<float3>& in_cloud);
	void from_pclCloud(
			const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud);

	unsigned int size() const;

	const pcl::PointXYZ& get_pcl(unsigned int i) const;
	void set_pcl(unsigned int i, const pcl::PointXYZ& point);

	// TODO Since the implementation is with PCL when returning the other
	// types vector we have to return a copy. This is quite inefficient
	// better find a way to avoid all this memory creation/destruction
	// This could be partially solved by using our own point type
	// that encapsulates all points types
	const osg::Vec3 get_osg(unsigned int i) const;
	void set_osg(unsigned int i, const osg::Vec3& point);

	//Method for an organised point cloud
	//Do not call on 1 row clouds
	const osg::Vec3 get_osg(unsigned int row, unsigned int column) const;
	void set_osg(unsigned int row, unsigned int column, const osg::Vec3& point);

	const float3 get_float3(unsigned int i) const;
	void set_float3(unsigned int i, const float3& point);

	const cv::Point3f get_point3(unsigned int i) const;
	void set_point3(unsigned int i, const cv::Point3f& point);

	float get_x(unsigned int i) const;
	void set_x(unsigned int i, float val);

	float get_y(unsigned int i) const;
	void set_y(unsigned int i, float val);

	float get_z(unsigned int i) const;
	void set_z(unsigned int i, float val);

	float length(unsigned int i);

	void push_back(const pcl::PointXYZ& point);
	void push_back(const osg::Vec3& point);
	void push_back(const float3& point);
	void push_back(const cv::Point3f& point);
	void push_back(float x, float y, float z);

	pcl::PointCloud<pcl::PointXYZ>::iterator begin();
	pcl::PointCloud<pcl::PointXYZ>::iterator end();
	pcl::PointXYZ& front();
	pcl::PointXYZ& back();

	const pcl::PointCloud<pcl::PointXYZ>::Ptr& get_cloud() const;

	//Resize for 1 row clouds
	void resize(size_t size);

	//Resize for row col clouds
	void resize(unsigned int height, unsigned int width);

	unsigned int get_width() const;
	unsigned int get_height() const;

	int save_to_file(const std::string& path);
	int load_from_file(const std::string& path);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

typedef boost::shared_ptr<PointCloud> PointCloudPtr;

#endif /* POINTCLOUD_H_ */

/*
 * PointCloud.cpp
 *
 *  Created on: 13 Mar 2014
 *      Author: m04701
 */

#include "PointCloud.h"

PointCloud::PointCloud() :
		cloud(new pcl::PointCloud<pcl::PointXYZ>) {
}

PointCloud::PointCloud(const osg::ref_ptr<osg::Vec3Array>& in_cloud) {
	from_osgVec3Array(in_cloud);
}
PointCloud::PointCloud(const std::vector<float3>& in_cloud) {
	from_float3Array(in_cloud);
}

PointCloud::PointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud) {
	from_pclCloud(in_cloud);
}

void PointCloud::from_osgVec3Array(
		const osg::ref_ptr<osg::Vec3Array>& in_cloud) {
	cloud->resize(in_cloud->size());
	for (unsigned int i = 0; i < in_cloud->size(); i++) {
		set_osg(i, in_cloud->at(i));
	}
}

void PointCloud::from_float3Array(const std::vector<float3>& in_cloud) {
	cloud->resize(in_cloud.size());
	for (unsigned int i = 0; i < in_cloud.size(); i++) {
		set_float3(i, in_cloud.at(i));
	}
}

void PointCloud::from_pclCloud(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud) {
	cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*in_cloud);
}

unsigned int PointCloud::size() const {
	return cloud->size();
}

const pcl::PointXYZ& PointCloud::get_pcl(unsigned int i) const {
	return cloud->points[i];
}

void PointCloud::set_pcl(unsigned int i, const pcl::PointXYZ& point) {
	cloud->points[i] = point;
}

const osg::Vec3 PointCloud::get_osg(unsigned int i) const {
	return osg::Vec3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
}

void PointCloud::set_osg(unsigned int i, const osg::Vec3& point) {
	cloud->points[i].x = point.x();
	cloud->points[i].y = point.y();
	cloud->points[i].z = point.z();
}

const float3 PointCloud::get_float3(unsigned int i) const {
	return make_float3(cloud->points[i].x, cloud->points[i].y,
			cloud->points[i].z);
}

void PointCloud::set_float3(unsigned int i, const float3& point) {
	cloud->points[i].x = point.x;
	cloud->points[i].y = point.y;
	cloud->points[i].z = point.z;
}

const cv::Point3f PointCloud::get_point3(unsigned int i) const {
	return cv::Point3f(cloud->points[i].x, cloud->points[i].y,
			cloud->points[i].z);
}
void PointCloud::set_point3(unsigned int i, const cv::Point3f& point) {
	cloud->points[i].x = point.x;
	cloud->points[i].y = point.y;
	cloud->points[i].z = point.z;
}

float PointCloud::get_x(unsigned int i) const {
	return cloud->points[i].x;
}

void PointCloud::set_x(unsigned int i, float val) {
	cloud->points[i].x = val;
}

float PointCloud::get_y(unsigned int i) const {
	return cloud->points[i].y;
}

void PointCloud::set_y(unsigned int i, float val) {
	cloud->points[i].y = val;
}

float PointCloud::get_z(unsigned int i) const {
	return cloud->points[i].z;
}

void PointCloud::set_z(unsigned int i, float val) {
	cloud->points[i].z = val;
}

float PointCloud::length(unsigned int i) {
	return sqrt(
			cloud->points[i].x * cloud->points[i].x
					+ cloud->points[i].y * cloud->points[i].y
					+ cloud->points[i].z * cloud->points[i].z);
}

void PointCloud::push_back(const pcl::PointXYZ& point) {
	cloud->push_back(point);
}

void PointCloud::push_back(const osg::Vec3& point) {
	cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
}

void PointCloud::push_back(const float3& point) {
	cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
}

void PointCloud::push_back(const cv::Point3f& point) {
	cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
}

void PointCloud::push_back(float x, float y, float z) {
	cloud->push_back(pcl::PointXYZ(x, y, z));
}

pcl::PointCloud<pcl::PointXYZ>::iterator PointCloud::begin() {
	return cloud->begin();
}

pcl::PointCloud<pcl::PointXYZ>::iterator PointCloud::end() {
	return cloud->end();
}

pcl::PointXYZ& PointCloud::front() {
	return cloud->front();
}

pcl::PointXYZ& PointCloud::back() {
	return cloud->back();
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr& PointCloud::get_cloud() const {
	return cloud;
}

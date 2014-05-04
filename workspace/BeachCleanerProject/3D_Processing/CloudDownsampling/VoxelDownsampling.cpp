/*
 * VoxelDownsampling.cpp
 *
 *  Created on: 04/04/2014
 *      Author: rgap
 */

#include "VoxelDownsampling.h"

VoxelDownsampling::VoxelDownsampling() {
}

void VoxelDownsampling::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	std::cout << "PointCloud before filtering has: " << cloud->points.size()
			<< " data points." << std::endl;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud);
	std::cout << "PointCloud after filtering has: " << cloud->points.size()
			<< " data points." << std::endl;
}

VoxelDownsampling::~VoxelDownsampling() {
}

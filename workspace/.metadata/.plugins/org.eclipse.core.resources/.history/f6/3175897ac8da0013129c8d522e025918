/*
 * VoxelDownsampling.h
 *
 *  Created on: 04/04/2014
 *      Author: rgap
 */

#ifndef VOXELDOWNSAMPLING_H_
#define VOXELDOWNSAMPLING_H_

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class VoxelDownsampling {
private:
	pcl::VoxelGrid<pcl::PointXYZ> vg;
public:
	VoxelDownsampling();
	void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	virtual ~VoxelDownsampling();
};

#endif /* VOXELDOWNSAMPLING_H_ */

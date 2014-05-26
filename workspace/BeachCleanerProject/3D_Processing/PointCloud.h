/*
 * PointCloud.h
 *
 *  Created on: 09/10/2013
 *      Author: rgap
 */

#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <opencv2/opencv.hpp>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "../globals.h"

using namespace cv;

/*! \class Abstract PointCloud class
 *  \brief Point cloud and viewer
 */

class PointCloudGrayscale;
class PointCloudRGB;

#define PCType PointCloudGrayscale

class PointCloud {
public:
	virtual void dumbFill() = 0;
	virtual void fillCloud(Mat &imgColor, Mat &imgDepth, Mat &imgFrameBin,
			float maxDepthDetected) = 0;
	void visualizeCloud();

protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	Size sizeWorld;

	PointCloud();
	PointCloud(Size& sizeWorld_);
	virtual ~PointCloud();
};

#endif /* POINTCLOUD_H_ */

/*
 * PointCloudGrayscale.h
 *
 *  Created on: 17/01/2014
 *      Author: rgap
 */

#ifndef POINTCLOUDGRAYSCALE_H_
#define POINTCLOUDGRAYSCALE_H_

#include "PointCloud.h"

/*! \class Concrete PointCloudGrayscale class
 *  \brief Point cloud and viewer
 */

class PointCloudGrayscale: public PointCloud {
public:
	PointCloudGrayscale();
	PointCloudGrayscale(Size &sizeWorld_);
	void dumbFill();
	void fillCloud(Mat &imgGrayscale, Mat &imgDepth, Mat &imgFrameBin, float maxDepthDetected = -1);

	virtual ~PointCloudGrayscale();

public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

};

#endif /* POINTCLOUDGRAYSCALE_H_ */

/*
 * PointCloudRGB.h
 *
 *  Created on: 17/01/2014
 *      Author: rgap
 */

#ifndef POINTCLOUDRGB_H_
#define POINTCLOUDRGB_H_

#include "PointCloud.h"

/*! \class Concrete PointCloudRGB class
 *  \brief Point cloud and viewer
 */

class PointCloudRGB: public PointCloud {
public:
	PointCloudRGB();
	PointCloudRGB(Size &sizeWorld_);
	void dumbFill();
	void fillCloud(Mat &imgBGR, Mat &imgDepth, Mat &imgFrameBin, float maxDepthDetected = -1);

	PCType* getPointCloud();

	~PointCloudRGB();
private:

	uint32_t rgb;
	pcl::PointXYZRGB point;

private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

#endif /* POINTCLOUDRGB_H_ */

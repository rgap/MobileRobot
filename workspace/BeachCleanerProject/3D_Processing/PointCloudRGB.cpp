/*
 * PointCloudRGB.cpp
 *
 *  Created on: 17/01/2014
 *      Author: rgap
 */

#include "PointCloudRGB.h"

using namespace cv;

PointCloudRGB::PointCloudRGB() {
}

PointCloudRGB::PointCloudRGB(Size &sizeWorld_) {
	sizeWorld = sizeWorld_;

	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width = sizeWorld.width; //Dimensions must be initialized to use 2-D indexing
	cloud->height = sizeWorld.height;
	cloud->is_dense = false;
	cloud->resize(sizeWorld.width * sizeWorld.height);

	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
			new pcl::visualization::PCLVisualizer("Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "cloud");
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 3,
			"cloud");
	viewer->addCoordinateSystem(MAX_DEPTH,30);
	viewer->initCameraParameters();
}

void PointCloudRGB::dumbFill() {
}


void PointCloudRGB::fillCloud(Mat &imgBGR, Mat &imgDepth, Mat &imgFrameBin, float maxDepthDetected) {

	float n = 1.0;
	if(maxDepthDetected >= 0){
		n = MAX_DEPTH/maxDepthDetected;
	}

	int i, j, k = 0;
	for (i = 0; i < imgDepth.rows; i++) { //2-D indexing
		uchar*imgBGR_ptr = imgBGR.ptr<uchar> (i);
		uchar*imgFrameBin_ptr = imgFrameBin.ptr<uchar> (i);
		Vec3f *imgDepth_ptr = imgDepth.ptr<Vec3f>(i);
		for (j = 0; j < imgDepth.cols; j++) {
			point.x = j;
			point.y = i;
			if (imgFrameBin_ptr[j] == 255) {
				point.z = imgDepth_ptr[j][2]*n;
				rgb = (static_cast<uint32_t> (imgBGR_ptr[3 * j + 0]) << 16
						| static_cast<uint32_t> (imgBGR_ptr[3 * j + 1]) << 8
						| static_cast<uint32_t> (imgBGR_ptr[3 * j + 2]));
				point.rgb = *reinterpret_cast<float*> (&rgb);
				cloud->points.push_back(point);
			} else {
				point.z = MAX_DEPTH;
				rgb = (static_cast<uint32_t> (255) << 16
						| static_cast<uint32_t> (255) << 8
						| static_cast<uint32_t> (255));
				point.rgb = *reinterpret_cast<float*> (&rgb);
				cloud->points.push_back(point);
			}
			k++;
		}
	}
	viewer->updatePointCloud(cloud, "cloud");
}


PointCloudRGB::~PointCloudRGB() {
}

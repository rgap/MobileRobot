/*
 * PointCloudGrayscale.cpp
 *
 *  Created on: 17/01/2014
 *      Author: rgap
 */

#include "PointCloudGrayscale.h"

using namespace cv;

PointCloudGrayscale::PointCloudGrayscale() {
}

PointCloudGrayscale::PointCloudGrayscale(Size &sizeWorld_) {
	sizeWorld = sizeWorld_;

	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
			new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = sizeWorld.width;
	cloud->height = sizeWorld.height;
	cloud->is_dense = false;
	cloud->resize(sizeWorld.width * sizeWorld.height);

	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
			new pcl::visualization::PCLVisualizer("Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 3,
			"cloud");
	viewer->addCoordinateSystem(MAX_DEPTH, 30);
	viewer->initCameraParameters();
}

void PointCloudGrayscale::dumbFill() {
	size_t i, j, k = 0;
	for (i = 0; i < sizeWorld.width; ++i) {
		for (j = 0; j < sizeWorld.width; ++j) {
			cloud->points[k].x = i;
			cloud->points[k].y = j;
			cloud->points[k].z = 5;
			k++;
		}
	}
	k = 0;
	for (i = 0; i < sizeWorld.width; ++i) {
		for (j = 0; j < sizeWorld.width; ++j) {
			if(k > cloud->size()/2){
				cloud->points[k].z = 200;
			}
			k++;
		}
	}
	viewer->updatePointCloud(cloud, "cloud");
}

void PointCloudGrayscale::fillCloud(Mat &imgGrayscale, Mat &imgDepth,
		Mat &imgFrameBin, float maxDepthDetected) {

	float n = 1.0;
	if (maxDepthDetected >= 0) {
		n = MAX_DEPTH / maxDepthDetected;
	}

	int i, j, k = 0;
	for (i = 0; i < imgDepth.rows; i++) { //2-D indexing
		uchar *imgFrameBin_ptr = imgFrameBin.ptr<uchar> (i);
		Vec3f *imgDepth_ptr = imgDepth.ptr<Vec3f> (i);
		for (j = 0; j < imgDepth.cols; j++) {
			cloud->points[k].x = imgDepth_ptr[j][0] * n;
			cloud->points[k].y = imgDepth_ptr[j][1] * n;
			if (imgFrameBin_ptr[j] == 255) {
				cloud->points[k].z = imgDepth_ptr[j][2] * n;
			} else {
				cloud->points[k].z = MAX_DEPTH;
			}
			k++;
		}
	}
	viewer->updatePointCloud(cloud);
}

PointCloudGrayscale::~PointCloudGrayscale() {
}

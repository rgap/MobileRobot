/*
 * RobotVision.cpp
 *
 *  Created on: 30/12/2013
 *      Author: rgap
 */

#include "RobotVision.h"
#include "globals.h"

RobotVision::RobotVision() {
	preprocess = new PreProcess();
	canRecognition = new CanRecognition();
	////////////////////////////////
	depthSensing = new DepthSensing();
	//patternRecognition = new PatternRecognition();
}

void RobotVision::fillPointCloud(Mat& imgColor, Mat& world, Mat &imgMaxDepth) {
	depthSensing->fillPointCloud(imgColor, world, imgMaxDepth);
}

void RobotVision::applyCloudDownsampling() {
	depthSensing->cloudDownsampling();
}

void RobotVision::applyObstacleRecognition(Mat& imgColor, Mat& world,
		Mat& notObstaclesBin, Mat& imgObstaclesBin) {
	imgObstaclesBin.setTo(cv::Scalar(0));
	depthSensing->segmentObstacles(world, notObstaclesBin,
			imgMaxDepth, imgObstaclesBin, imgPlane, P->maxDistObstacle,
			P->maxHeightFloor, P->distThreshold, P->epsAngle, P->axisVal);
	depthSensing->visualizePointCloud();
}

void RobotVision::applyGaussBlur(Mat &imgIni, Mat &imgRes) {
	preprocess->gaussBlur(imgIni, imgRes, P->blurSize, P->blurSize);
}

void RobotVision::applyBilateralFilter(Mat &imgIni, Mat &imgRes) {
	preprocess->bilateralFilter_repet(imgIni, imgRes, 5, 5, 9, 7);
}

void RobotVision::applySSR(Mat &imgIni, Mat &imgRes) {
	preprocess->mejoramientoSSR(imgIni, imgRes, P->sigma, P->alpha, P->beta);
}

void RobotVision::applyColorSegmentation_HSV(Mat &imgIni, Mat &imgRes) {
	canRecognition->segmentation_HSV(imgIni, imgRes, P->minH_can, P->maxH_can,
			P->minS_can, P->maxS_can, P->minV_can, P->maxV_can);
}

void RobotVision::applyShapeSegmentation(Mat &imgIni, Mat &imgRes) {
	imgSegShape_Can.setTo(cv::Scalar(0));
	canRecognition->shapeSegmentation(imgIni, imgRes, world, imgMaxDepth,
			imgBGR, P->minArea, P->maxArea, P->minApCuadrado, P->maxApCuadrado);
}

void RobotVision::applyDepthThresholding(Mat &world, Mat &imgMaxDepth) {
	imgMaxDepth.setTo(cv::Scalar(0));
	depthSensing->depthThresholding(world, imgMaxDepth, P->max_depth);
}

//////////////////////// ENVIRONMENT
Point3f *RobotVision::getPointNearestCan() {
	return canRecognition->getPointNearestCan();
}

void RobotVision::getColorizeDisparity(Mat &disparityMap) {
	depthSensing->colorizeDisparity(disparityMap, disparityColorized, 50);
	disparityColorized.copyTo(disparityColorized, disparityMap != 0);
}

void RobotVision::initializeVision(cv::Size &szFrame) {
	//cv::namedWindow("I");
	//cv::namedWindow("imgEnhanced_blur");
	//cv::namedWindow("I_ROIS");
	//cv::namedWindow("imgMaxDepth");

	imgBGR_valid = cv::Mat(szFrame, CV_8UC3, cv::Scalar(0));
	imgBGR_Enh = cv::Mat(szFrame, CV_8UC3, cv::Scalar(0));
	imgBGR_Blur = cv::Mat(szFrame, CV_8UC3, cv::Scalar(0));

	//////////////////////////////////// SEGMENTATION
	imgBGR_Can = cv::Mat(szFrame, CV_8UC3, cv::Scalar(0));
	imgSegColor_Can = cv::Mat(szFrame, CV_8UC1, cv::Scalar(0));
	imgMaxDepth = cv::Mat(szFrame, CV_8UC1, cv::Scalar(0));

	imgThresh_momentos = cv::Mat(szFrame, CV_8UC1, cv::Scalar(0));
	imgThresh_resClasif = cv::Mat(szFrame, CV_8UC1, cv::Scalar(0));
	imgObstaclesBin = cv::Mat(szFrame, CV_8UC1, cv::Scalar(0));
	imgPlane = cv::Mat(szFrame, CV_8UC1, cv::Scalar(0));
	imgSegShape_Can = cv::Mat(szFrame, CV_8UC1, cv::Scalar(0));

	depthSensing->initializePointCloud(szFrame);
	preprocess->initializeMats(szFrame);
}

void RobotVision::setParameters(ModelParameters* P_) {
	P = P_;
}

RobotVision::~RobotVision() {
}

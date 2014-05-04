/*
 * DeviceCapture.h
 *
 *  Created on: 30/12/2013
 *      Author: rgap
 */

#ifndef DEVICECAPTURE_H_
#define DEVICECAPTURE_H_

#include <opencv2/opencv.hpp>

class DeviceCapture {
private:
	cv::Mat imgBGR;
	cv::Mat world;

	int CAPTURE_TYPE_;

public:
	cv::Size szFrame;
	cv::VideoCapture capture;

	DeviceCapture(int CAPTURE_TYPE, const char*path_img_bgr = NULL, const char*path_img_depth = NULL);
	void verifyState();
	cv::Mat getBGRFrame();
	cv::Mat getDepthFrame();
	void showCameraFeatures();

	float getMaxDepth();
	virtual ~DeviceCapture();
};

#endif /* DEVICECAPTURE_H_ */

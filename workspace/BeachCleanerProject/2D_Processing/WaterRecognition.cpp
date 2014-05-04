/*
 * WaterRecognition.cpp
 *
 *  Created on: 31/12/2013
 *      Author: rgap
 */

#include "WaterRecognition.h"

WaterRecognition::WaterRecognition() {
}

void WaterRecognition::segmentation_HSV(Mat &imgRGB, Mat &grisesObstaculos, int minH,
		int maxH, int minS, int maxS, int minV, int maxV) {
	cvtColor(imgRGB, imgHSV, CV_BGR2HSV);
	int i, j;
	for (i = 0; i < imgRGB.rows; ++i) {
		uchar*imgCONV_pt = imgHSV.ptr<uchar> (i);
		uchar*imgTemp_pt = grisesObstaculos.ptr<uchar> (i);
		for (j = 0; j < imgRGB.cols; ++j) {
			if (imgCONV_pt[3 * j + 0] >= minH && imgCONV_pt[3 * j + 0] <= maxH
					&& imgCONV_pt[3 * j + 1] >= minS && imgCONV_pt[3 * j + 1]
					<= maxS && imgCONV_pt[3 * j + 2] >= minV && imgCONV_pt[3
					* j + 2] <= maxV) {
				imgTemp_pt[j] = 255;
				imgTemp_pt[j] = 255;
				imgTemp_pt[j] = 255;
			} else {
				imgTemp_pt[j] = 0;
				imgTemp_pt[j] = 0;
				imgTemp_pt[j] = 0;
			}
		}
	}
}

WaterRecognition::~WaterRecognition() {
}

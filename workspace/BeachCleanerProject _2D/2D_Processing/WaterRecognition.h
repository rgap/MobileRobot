/*
 * WaterRecognition.h
 *
 *  Created on: 31/12/2013
 *      Author: rgap
 */

#ifndef WATERRECOGNITION_H_
#define WATERRECOGNITION_H_

#include <opencv2/opencv.hpp>

using namespace cv;

class WaterRecognition {
private:
	Mat imgHSV; /**< Usada en Segmentation_HSV */
public:
	WaterRecognition();
	void segmentation_HSV(Mat &imgRGB, Mat &grisesObstaculos, int minH,
			int maxH, int minS, int maxS, int minV, int maxV);
	virtual ~WaterRecognition();
};

#endif /* WATERRECOGNITION_H_ */

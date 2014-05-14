/*
 * CanRecognition.h
 *
 *  Created on: 06/10/2013
 *      Author: rgap
 */

#ifndef CAN_RECOGNITION_H_
#define CAN_RECOGNITION_H_

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/*! \class Class CanRecognition - CanRecognition.h
 *  \brief Segmentacion de latas
 */

class CanRecognition {

private:
	Mat imgHSV; /**< Usada en Segmentation_HSV */
	Mat imgTemp;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	int can_found;
	Point3f* pointNearestCan;

public:

	CanRecognition();
	Point3f *getPointNearestCan();

	/**
	 * Calcular region de la lata mas cercana,
	 */
	void shapeSegmentation(Mat& imgSegColor_Can, Mat& imgSegShape_Can,
			Mat &world, Mat &imgMaxDepth, Mat &imgBGR_cansDetected,
			int minArea, int maxArea, float minApCuadrado, float maxApCuadrado);

	/**
	 * Segmentacion de color en el espacio HSV
	 */
	void segmentation_HSV(Mat &imgRGB, Mat &imgThreshold, int minH, int maxH,
			int minS, int maxS, int minV, int maxV);

	~CanRecognition();
};

#endif /* CAN_RECOGNITION_H_ */

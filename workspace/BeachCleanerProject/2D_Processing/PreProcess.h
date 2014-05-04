/*
 * PreProcess.h
 *
 *  Created on: 06/10/2013
 *      Author: rgap
 */

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

#include "Retinex.h"

using namespace std;
using namespace cv;

class PreProcess {
private:
	Retinex *retinex;

public:
	PreProcess();
	void initializeMats(Size& szImg);
	void gaussBlur(Mat&img_ini, Mat&img_res, int krows, int kcols);
	void mejoramientoSSR(Mat&img, Mat&imgEnhanced, float sigma, int alpha,
			int beta);
	void bilateralFilter_repet(Mat&img, Mat& imgRes, int repetitions, int size,
			double sigmaColor, double sigmaSpace);

};

#endif /* PREPROCESS_H_ */

/*
 * Retinex.h
 *
 *  Created on: 06/10/2013
 *      Author: rgap
 */

#ifndef RETINEX_H_
#define RETINEX_H_

using namespace std;
using namespace cv;

# define RETINEX_UNIFORM 0
# define RETINEX_LOW     1
# define RETINEX_HIGH    2

class Retinex {

public:

	Mat imgBlur;
	Mat imgEnhanced_float;
	Size szTmp;

	//////////////////////
	Retinex();

	void initializeMats(Size& szImg);
	void distribute_scales(float* scales, int nscales, int mode, int s);
	void distribute_weights(float* weights, int num_scales);

	void SSR(cv::Mat &img, cv::Mat &imgEnhanced, float sigma, float gain = 1,
			float offset = 0);

	void SSR_blur(cv::Mat &img, cv::Mat &imgEnhanced, float sigma,
			float gain = 1, float offset = 0);

	void MSR(Mat &imgMat, Mat &imgEnhancedMat, int scales, float *weights,
			float *sigmas, int gain, int offset);

	void MSR_blur(Mat &imgMat, Mat &imgEnhancedMat, int scales, float *weights,
			float *sigmas, int gain, int offset);

	void MSRCR(Mat &imgMat, Mat &imgEnhancedMat, int num_scales,
			float *weights, float *sigmas, float alpha, float beta, float gain,
			float offset);

	void MSRCR_blur(Mat &imgMat, Mat &imgEnhancedMat, int num_scales,
			float *weights, float *sigmas, float alpha, float beta, float gain,
			float offset);

};

#endif /* RETINEX_H_ */

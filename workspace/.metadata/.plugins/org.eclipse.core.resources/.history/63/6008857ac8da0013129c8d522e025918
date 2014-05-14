#include "PreProcess.h"

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

using namespace cv;

PreProcess::PreProcess() {
	retinex = new Retinex();
}

void PreProcess::initializeMats(Size& szImg) {
	retinex->initializeMats(szImg);
}

void PreProcess::gaussBlur(Mat&img_ini, Mat&img_res, int krows, int kcols) {
	GaussianBlur(img_ini, img_res, Size(krows, kcols), 0);
}

void PreProcess::mejoramientoSSR(Mat&img, Mat&imgEnhanced, float sigma, int alpha, int beta) {
	retinex->SSR_blur(img, imgEnhanced, sigma, alpha, beta);
}

void PreProcess::bilateralFilter_repet(Mat&img, Mat& imgRes,
		int repetitions, int size, double sigmaColor, double sigmaSpace) {

	// Perform many iterations of weak bilateral filtering, to enhance the edges while blurring the flat regions, like a cartoon.
	Mat tmp = img.clone();

	for (int i = 0; i < repetitions; i++) {
		bilateralFilter(tmp, imgRes, size, sigmaColor,
				sigmaSpace);
		tmp = imgRes.clone();
	}
}

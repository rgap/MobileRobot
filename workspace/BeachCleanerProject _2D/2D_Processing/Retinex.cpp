#include <opencv2/opencv.hpp>
#include "Retinex.h"

using namespace cv;

Retinex::Retinex() {
}

void Retinex::initializeMats(Size& szImg) {
	imgEnhanced_float = Mat(szImg, CV_32FC3);
	imgBlur = Mat(szImg, CV_8UC3);
	szTmp = Size();
}

void Retinex::distribute_weights(float* weights, int num_scales) {
	float weight_val = 1.0f / (float) num_scales;
	for (int i = 0; i < num_scales; ++i)
		weights[i] = weight_val;
}

void Retinex::distribute_scales(float* scales, int num_scales, int mode, int s) {
	if (num_scales == 1) { // For one filter we choose the median scale
		scales[0] = (float) s / 2;
	} else if (num_scales == 2) { // For two filters we choose the median and maximum scale
		scales[0] = (float) s / 2;
		scales[1] = (float) s;
	} else {
		float size_step = (float) s / (float) num_scales;
		int i;
		switch (mode) {
		case RETINEX_UNIFORM:
			for (i = 0; i < num_scales; ++i)
				scales[i] = 2.0f + (float) i * size_step;
			break;
		case RETINEX_LOW:
			size_step = (float) log(s - 2.0f) / (float) num_scales;
			for (i = 0; i < num_scales; ++i)
				scales[i] = 2.0f + (float) pow(10, (i * size_step) / log(10));
			break;
		case RETINEX_HIGH:
			size_step = (float) log(s - 2.0) / (float) num_scales;
			for (i = 0; i < num_scales; ++i)
				scales[i] = s - (float) pow(10, (i * size_step) / log(10));
			break;
		default:
			break;
		}
	}
}

void Retinex::SSR_blur(Mat &imgMat, Mat &imgEnhancedMat, float sigma,
		float gain, float offset) {
	int i, j, k, filter_size;

	filter_size = (int) floor(sigma * 6) / 2;
	filter_size = filter_size * 2 + 1;

	blur(imgMat, imgBlur, Size(filter_size, filter_size), Point(-1, -1));
	//imshow("blurred", imgBlur);

	// R_SSR(x, y) = log(I(x,y)) - log[F(x,y)*I(x,y)]
	for (i = 0; i < imgMat.rows; i++) {
		uchar*imgMat_pt = imgMat.ptr<uchar> (i);
		uchar*imgBlur_pt = imgBlur.ptr<uchar> (i);
		float* imgEnhanced_float_pt = imgEnhanced_float.ptr<float> (i);

		for (j = 0; j < imgMat.cols; j++) {
			for (k = 0; k < imgMat.channels(); k++) {
				imgEnhanced_float_pt[3 * j + k] = log(
						(imgMat_pt[3 * j + k] + 1.0f) * 1.0f / (imgBlur_pt[3 * j
								+ k] + 1.0f) + 0.5f);
			}
		}
	}
	imgEnhanced_float.convertTo(imgEnhancedMat, CV_8UC3, gain, offset);
}

void Retinex::SSR(Mat &imgMat, Mat &imgEnhancedMat, float sigma, float gain,
		float offset) {
	int i, j, k;
	// R_MSR(x, y) =  SUM(W * R_SSR(x, y))
	//GaussianBlur(imgMat, imgBlur, Size(201,201), 0);

	//boxFilter(imgMat, imgBlur, -1, Size(201,201));

	//medianBlur(imgMat, imgBlur, 201);
	blur(imgMat, imgBlur, Size(300, 300), Point(-1, -1));
	imshow("blurred", imgBlur);

	// R_SSR(x, y) = log(I(x,y)) - log[F(x,y)*I(x,y)]
	for (i = 0; i < imgMat.rows; i++) {
		uchar*imgMat_pt = imgMat.ptr<uchar> (i);
		uchar*imgBlur_pt = imgBlur.ptr<uchar> (i);
		float* imgEnhanced_float_pt = imgEnhanced_float.ptr<float> (i);

		for (j = 0; j < imgMat.cols; j++) {
			for (k = 0; k < imgMat.channels(); k++) {
				imgEnhanced_float_pt[3 * j + k] = log(
						(imgMat_pt[3 * j + k] + 1.0f) * 1.0 / (imgBlur_pt[3 * j
								+ k] + 1.0f));
			}
		}
	}
	imgEnhanced_float.convertTo(imgEnhancedMat, CV_8UC3, gain, offset);
}

void Retinex::MSR_blur(Mat &imgMat, Mat &imgEnhancedMat, int num_scales,
		float *weights, float *sigmas, int gain, int offset) {

	int scale, i, j, k, init_imgEnh = 1, filter_size;

	// R_MSR(x, y) =  SUM(W * R_SSR(x, y))
	for (scale = 0; scale < num_scales; ++scale) { // MSR

		filter_size = (int) floor(sigmas[scale] * 6) / 2;
		filter_size = filter_size * 2 + 1;
		cout << "filter_size: " << filter_size << endl;
		blur(imgMat, imgBlur, Size(filter_size, filter_size), Point(-1, -1));

		// R_SSR(x, y) = log(I(x,y)) - log[F(x,y)*I(x,y)]
		for (i = 0; i < imgMat.rows; ++i) {
			uchar*imgMat_pt = imgMat.ptr<uchar> (i);
			uchar*imgBlur_pt = imgBlur.ptr<uchar> (i);
			float* imgEnhanced_float_pt = imgEnhanced_float.ptr<float> (i);

			for (j = 0; j < imgMat.cols; ++j) {

				for (k = 0; k < imgMat.channels(); ++k) {
					if (init_imgEnh)
						imgEnhanced_float_pt[3 * j + k] = 0;

					imgEnhanced_float_pt[3 * j + k] += weights[scale] * log(
							(imgMat_pt[3 * j + k] + 1.0f) * 1.0 / (imgBlur_pt[3
									* j + k] + 1.0f));
				}
			}
		}
		init_imgEnh = 0;
	}

	imgEnhanced_float.convertTo(imgEnhancedMat, CV_8UC3, gain, offset);
}

void Retinex::MSR(Mat &imgMat, Mat &imgEnhancedMat, int num_scales,
		float *weights, float *sigmas, int gain, int offset) {
	int scale, i, j, k, init_imgEnh = 1;

	// R_MSR(x, y) =  SUM(W * R_SSR(x, y))
	for (scale = 0; scale < num_scales; ++scale) { // MSR

		GaussianBlur(imgMat, imgBlur, szTmp, sigmas[scale]);

		// R_SSR(x, y) = log(I(x,y)) - log[F(x,y)*I(x,y)]
		for (i = 0; i < imgMat.rows; ++i) {
			uchar*imgMat_pt = imgMat.ptr<uchar> (i);
			uchar*imgBlur_pt = imgBlur.ptr<uchar> (i);
			float* imgEnhanced_float_pt = imgEnhanced_float.ptr<float> (i);

			for (j = 0; j < imgMat.cols; ++j) {

				for (k = 0; k < imgMat.channels(); ++k) {
					if (init_imgEnh)
						imgEnhanced_float_pt[3 * j + k] = 0;

					imgEnhanced_float_pt[3 * j + k] += weights[scale] * log(
							(imgMat_pt[3 * j + k] + 1.0f) * 1.0 / (imgBlur_pt[3
									* j + k] + 1.0f));
				}
			}
		}
		init_imgEnh = 0;
	}

	imgEnhanced_float.convertTo(imgEnhancedMat, CV_8UC3, gain, offset);
}

void Retinex::MSRCR_blur(Mat &imgMat, Mat &imgEnhancedMat, int num_scales,
		float *weights, float *sigmas, float alpha, float beta, float gain,
		float offset) {

	int scale, i, j, k, init_imgEnh = 1, filter_size;
	float sum, Cval;

	// R_MSR(x, y) =  SUM(W * R_SSR(x, y))
	for (scale = 0; scale < num_scales; ++scale) { // MSR

		filter_size = (int) floor(sigmas[scale] * 6) / 2;
		filter_size = filter_size * 2 + 1;
		cout << filter_size << " ";
		//cout << "filter_size: " << filter_size << endl;

		boxFilter(imgMat, imgBlur, -1, Size(filter_size, filter_size),
				Point(-1, -1), true);
		//medianBlur(imgMat, imgBlur, filter_size);
		//blur(imgMat, imgBlur, Size(filter_size, filter_size), Point(-1, -1));

		// R_SSR(x, y) = log(I(x,y)) - log[F(x,y)*I(x,y)]
		for (i = 0; i < imgMat.rows; ++i) {
			uchar*imgMat_pt = imgMat.ptr<uchar> (i);
			uchar*imgBlur_pt = imgBlur.ptr<uchar> (i);
			float* imgEnhanced_float_pt = imgEnhanced_float.ptr<float> (i);

			for (j = 0; j < imgMat.cols; ++j) {

				sum = imgMat_pt[3 * j + 0] + imgMat_pt[3 * j + 1] + imgMat_pt[3
						* j + 2];

				for (k = 0; k < imgMat.channels(); ++k) {
					if (init_imgEnh)
						imgEnhanced_float_pt[3 * j + k] = 0;

					Cval = beta * log(
							alpha * imgMat_pt[3 * j + k] * 1.0 / sum + 1.0f);

					imgEnhanced_float_pt[3 * j + k] += Cval * weights[scale]
							* log(
									((imgMat_pt[3 * j + k] + 1.0f) * 1.0)
											/ (imgBlur_pt[3 * j + k] + 1.0f));
				}
			}
		}
		init_imgEnh = 0;
	}
	cout << endl;

	imgEnhanced_float.convertTo(imgEnhancedMat, CV_8UC3, gain, offset);
}

void Retinex::MSRCR(Mat &imgMat, Mat &imgEnhancedMat, int num_scales,
		float *weights, float *sigmas, float alpha, float beta, float gain,
		float offset) {

	int scale, i, j, k, init_imgEnh = 1;
	float sum, Cval;
	// R_MSR(x, y) =  SUM(W * R_SSR(x, y))
	for (scale = 0; scale < num_scales; ++scale) { // MSR

		GaussianBlur(imgMat, imgBlur, szTmp, sigmas[scale]);

		// R_SSR(x, y) = log(I(x,y)) - log[F(x,y)*I(x,y)]
		for (i = 0; i < imgMat.rows; ++i) {
			uchar*imgMat_pt = imgMat.ptr<uchar> (i);
			uchar*imgBlur_pt = imgBlur.ptr<uchar> (i);
			float* imgEnhanced_float_pt = imgEnhanced_float.ptr<float> (i);

			for (j = 0; j < imgMat.cols; ++j) {

				sum = imgMat_pt[3 * j + 0] + imgMat_pt[3 * j + 1] + imgMat_pt[3
						* j + 2];

				for (k = 0; k < imgMat.channels(); ++k) {
					if (init_imgEnh)
						imgEnhanced_float_pt[3 * j + k] = 0;

					Cval = beta * log(
							alpha * imgMat_pt[3 * j + k] * 1.0 / sum + 1.0f);

					imgEnhanced_float_pt[3 * j + k] += Cval * weights[scale]
							* log(
									((imgMat_pt[3 * j + k] + 1.0f) * 1.0)
											/ (imgBlur_pt[3 * j + k] + 1.0f));
				}
			}
		}
		init_imgEnh = 0;
	}

	imgEnhanced_float.convertTo(imgEnhancedMat, CV_8UC3, gain, offset);
}

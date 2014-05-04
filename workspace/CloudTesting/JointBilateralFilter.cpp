/*
 * JointBilateralFilter.cpp
 *
 *  Created on: 07/03/2014
 *      Author: rgap
 */

#include "JointBilateralFilter.h"

using namespace std;
using namespace cv;

void JointBilateralFilter::jointBilateralFilter_32f(const Mat& src, const Mat& guide, Mat& dst,
		Size kernelSize, double sigma_color, double sigma_space, int borderType) {
	if (kernelSize.width == 0 || kernelSize.height == 0) {
		src.copyTo(dst);
		return;
	}
	int cn = src.channels();
	int cng = guide.channels();
	int i, j, maxk;
	Size size = src.size();

	CV_Assert(
			(src.type() == CV_32FC1 || src.type() == CV_32FC3) && (guide.type()
					== CV_32FC1 || guide.type() == CV_32FC3) && src.type()
			== dst.type() && src.size() == dst.size())
		;

	if (sigma_color <= 0)
		sigma_color = 1;
	if (sigma_space <= 0)
		sigma_space = 1;

	double gauss_color_coeff = -0.5 / (sigma_color * sigma_color);
	double gauss_space_coeff = -0.5 / (sigma_space * sigma_space);

	int radiusH = kernelSize.width >> 1;
	int radiusV = kernelSize.height >> 1;

	Mat temp, tempg;

	int dpad = (4 - src.cols % 4) % 4;
	int spad = dpad + (4 - (2 * radiusH) % 4) % 4;
	if (spad < 4)
		spad += 4;
	int lpad = 4 * (radiusH / 4 + 1) - radiusH;
	int rpad = spad - lpad;
	if (cn == 1 && cng == 1) {
		copyMakeBorder(src, temp, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);
		copyMakeBorder(guide, tempg, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);
	} else if (cn == 1 && cng == 3) {
		copyMakeBorder(src, temp, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);
		Mat temp2;
		copyMakeBorder(guide, temp2, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);
	} else if (cn == 3 && cng == 3) {
		Mat temp2;
		copyMakeBorder(src, temp2, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);

		copyMakeBorder(guide, temp2, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);
	} else if (cn == 3 && cng == 1) {
		Mat temp2;
		copyMakeBorder(src, temp2, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);

		copyMakeBorder(guide, tempg, radiusV, radiusV, radiusH + lpad,
				radiusH + rpad, borderType);
	}

	double minv, maxv;
	minMaxLoc(guide, &minv, &maxv);
	const int color_range = cvRound(maxv - minv);

	vector<float> _color_weight(cng * color_range);
	vector<float> _space_weight(kernelSize.area() + 1);
	vector<int> _space_ofs(kernelSize.area() + 1);
	vector<int> _space_guide_ofs(kernelSize.area() + 1);
	float* color_weight = &_color_weight[0];
	float* space_weight = &_space_weight[0];
	int* space_ofs = &_space_ofs[0];
	int* space_guide_ofs = &_space_guide_ofs[0];

	// initialize color-related bilateral filter coefficients

	for (i = 0; i < color_range * cng; i++)
		color_weight[i] = (float) std::exp(i * i * gauss_color_coeff);

	// initialize space-related bilateral filter coefficients
	for (i = -radiusV, maxk = 0; i <= radiusV; i++) {
		j = -radiusH;

		for (; j <= radiusH; j++) {
			double r = std::sqrt((double) i * i + (double) j * j);
			if (r > max(radiusV, radiusH))
				continue;
			space_weight[maxk] = (float) std::exp(r * r * gauss_space_coeff);
			space_ofs[maxk] = (int) (i * temp.cols * cn + j);
			space_guide_ofs[maxk++] = (int) (i * tempg.cols * cng + j);
		}
	}

	Mat dest = Mat::zeros(Size(src.cols + dpad, src.rows), dst.type());
	JointBilateralFilter_32f_InvokerSSE4 body(dest, temp, tempg, radiusH,
			radiusV, maxk, space_ofs, space_guide_ofs, space_weight,
			color_weight);
	parallel_for_(Range(0, size.height), body);
	Mat(dest(Rect(0, 0, dst.cols, dst.rows))).copyTo(dst);
}



void JointBilateralFilter::jointBilateralFilterSP_32f(const Mat& src, const Mat& guide, Mat& dst,
		Size kernelSize, double sigma_color, double sigma_space, int borderType) {
	jointBilateralFilter_32f(src, guide, dst, Size(kernelSize.width, 1),
			sigma_color, sigma_space, borderType);
	jointBilateralFilter_32f(dst, guide, dst, Size(1, kernelSize.height),
			sigma_color, sigma_space, borderType);
}

void JointBilateralFilter::jointBilateralFilter(const Mat& src, const Mat& guide, Mat& dst,
		Size kernelSize, double sigma_color, double sigma_space, int method, int borderType) {
	if (dst.empty())
		dst.create(src.size(), src.type());
	if (method == BILATERAL_NORMAL) {
		if (src.type() == CV_MAKE_TYPE(CV_32F, src.channels()))
			jointBilateralFilter_32f(src, guide, dst, kernelSize, sigma_color,
					sigma_space, borderType);
	} else if (method == BILATERAL_SEPARABLE) {
		if (src.type() == CV_MAKE_TYPE(CV_32F, src.channels()))
			jointBilateralFilterSP_32f(src, guide, dst, kernelSize,
					sigma_color, sigma_space, borderType);
	}
}

/*
 * JointBilateralFilter.h
 *
 *  Created on: 07/03/2014
 *      Author: rgap
 */

#ifndef JOINTBILATERALFILTER_H_
#define JOINTBILATERALFILTER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/internal.hpp>

using namespace std;
using namespace cv;

enum {
	BILATERAL_NORMAL = 0, BILATERAL_SEPARABLE
};

class JointBilateralFilter_32f_InvokerSSE4: public cv::ParallelLoopBody {
public:
	JointBilateralFilter_32f_InvokerSSE4(Mat& _dest, const Mat& _temp,
			const Mat& _guide, int _radiusH, int _radiusV, int _maxk,
			int* _space_ofs, int* _space_guide_ofs, float *_space_weight,
			float *_color_weight) :
		temp(&_temp), dest(&_dest), guide(&_guide), radiusH(_radiusH),
				radiusV(_radiusV), maxk(_maxk), space_ofs(_space_ofs),
				space_guide_ofs(_space_guide_ofs), space_weight(_space_weight),
				color_weight(_color_weight) {
	}

	virtual void operator()(const Range& range) const {
		int i, j, cn = dest->channels(), k;
		int cng = (guide->rows - 2 * radiusV) / dest->rows;
		Size size = dest->size();

		static int CV_DECL_ALIGNED(16) v32f_absmask[] = {0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff};
		if (cn == 1 && cng == 1) {
			int CV_DECL_ALIGNED(16) buf[4];

			float* sptr = (float*) temp->ptr<float> (range.start + radiusV) + 4
					* (radiusH / 4 + 1);
			float* gptr = (float*) guide->ptr<float> (range.start + radiusV)
					+ 4 * (radiusH / 4 + 1);
			float* dptr = dest->ptr<float> (range.start);
			const int sstep = temp->cols;
			const int gstep = guide->cols;
			const int dstep = dest->cols;
			for (i = range.start; i != range.end; i++, dptr += dstep, sptr
					+= sstep, gptr += gstep) {
				j = 0;
				for (; j < size.width; j++) {
					const float val0 = gptr[j];
					float sum = 0.0f;
					float wsum = 0.0f;
					for (k = 0; k < maxk; k++) {
						float gval = gptr[j + space_guide_ofs[k]];
						float val = sptr[j + space_ofs[k]];
						float w = space_weight[k] * color_weight[cvRound(
								std::abs(gval - val0))];
						sum += val * w;
						wsum += w;
					}
					dptr[j] = sum / wsum;
				}
			}
		} else if (cn == 1 && cng == 3) {
			assert(cng == 3);//color
			int CV_DECL_ALIGNED(16) buf[4];

			const int sstep = temp->cols;
			const int gstep = 3 * guide->cols;
			const int dstep = dest->cols;

			float* sptr = (float*) temp->ptr<float> (range.start + radiusV) + 4
					* (radiusH / 4 + 1);
			float* gptrr = (float*) guide->ptr<float> (
					3 * radiusV + 3 * range.start) + 4 * (radiusH / 4 + 1);
			float* gptrg = (float*) guide->ptr<float> (
					3 * radiusV + 3 * range.start + 1) + 4 * (radiusH / 4 + 1);
			float* gptrb = (float*) guide->ptr<float> (
					3 * radiusV + 3 * range.start + 2) + 4 * (radiusH / 4 + 1);

			float* dptr = dest->ptr<float> (range.start);
			for (i = range.start; i != range.end; i++, gptrr += gstep, gptrg
					+= gstep, gptrb += gstep, sptr += sstep, dptr += dstep) {
				j = 0;

				for (; j < size.width; j++) {
					const float* sptrj = sptr + j;
					const float* gptrrj = gptrr + j;
					const float* gptrgj = gptrg + j;
					const float* gptrbj = gptrb + j;

					const float r0 = gptrrj[0];
					const float g0 = gptrgj[0];
					const float b0 = gptrbj[0];

					float sum = 0.0f;
					float wsum = 0.0f;
					for (k = 0; k < maxk; k++) {
						const float r = gptrrj[space_guide_ofs[k]], g =
								gptrgj[space_guide_ofs[k]], b =
								gptrbj[space_guide_ofs[k]];
						float w = space_weight[k] * color_weight[cvRound(
								std::abs(b - b0) + std::abs(g - g0) + std::abs(
										r - r0))];
						sum += sptrj[space_ofs[k]] * w;
						wsum += w;
					}
					dptr[j] = sum / wsum;
				}
			}
		} else if (cn == 3 && cng == 3) {
			assert(cng == 3);// color
			int CV_DECL_ALIGNED(16) buf[4];

			const int sstep = 3 * temp->cols;
			const int gstep = 3 * guide->cols;
			const int dstep = 3 * dest->cols;

			float* sptrr = (float*) temp->ptr<float> (
					3 * radiusV + 3 * range.start) + 4 * (radiusH / 4 + 1);
			float* sptrg = (float*) temp->ptr<float> (
					3 * radiusV + 3 * range.start + 1) + 4 * (radiusH / 4 + 1);
			float* sptrb = (float*) temp->ptr<float> (
					3 * radiusV + 3 * range.start + 2) + 4 * (radiusH / 4 + 1);
			float* gptrr = (float*) guide->ptr<float> (
					3 * radiusV + 3 * range.start) + 4 * (radiusH / 4 + 1);
			float* gptrg = (float*) guide->ptr<float> (
					3 * radiusV + 3 * range.start + 1) + 4 * (radiusH / 4 + 1);
			float* gptrb = (float*) guide->ptr<float> (
					3 * radiusV + 3 * range.start + 2) + 4 * (radiusH / 4 + 1);
			float* dptr = dest->ptr<float> (range.start);
			for (i = range.start; i != range.end; i++, gptrr += gstep, gptrg
					+= gstep, gptrb += gstep, sptrr += sstep, sptrg += sstep, sptrb
					+= sstep, dptr += dstep) {
				j = 0;

				for (; j < size.width; j++) {
					const float* sptrrj = sptrr + j;
					const float* sptrgj = sptrg + j;
					const float* sptrbj = sptrb + j;
					const float* gptrrj = gptrr + j;
					const float* gptrgj = gptrg + j;
					const float* gptrbj = gptrb + j;

					const float r0 = gptrrj[0];
					const float g0 = gptrgj[0];
					const float b0 = gptrbj[0];

					float sum_r = 0.0f, sum_b = 0.0f, sum_g = 0.0f;
					float wsum = 0.0f;
					for (k = 0; k < maxk; k++) {
						const float r = gptrrj[space_guide_ofs[k]], g =
								gptrgj[space_guide_ofs[k]], b =
								gptrbj[space_guide_ofs[k]];
						float w = space_weight[k] * color_weight[cvRound(
								std::abs(b - b0) + std::abs(g - g0) + std::abs(
										r - r0))];
						sum_b += sptrrj[space_ofs[k]] * w;
						sum_g += sptrgj[space_ofs[k]] * w;
						sum_r += sptrbj[space_ofs[k]] * w;
						wsum += w;
					}
					wsum = 1.f / wsum;
					dptr[3 * j] = sum_b * wsum;
					dptr[3 * j + 1] = sum_g * wsum;
					dptr[3 * j + 2] = sum_r * wsum;
				}
			}

		} else if (cn == 3 && cng == 1) {
			int CV_DECL_ALIGNED(16) buf[4];

			const int sstep = 3 * temp->cols;
			const int gstep = guide->cols;
			const int dstep = 3 * dest->cols;

			float* sptrr = (float*) temp->ptr<float> (
					3 * radiusV + 3 * range.start) + 4 * (radiusH / 4 + 1);
			float* sptrg = (float*) temp->ptr<float> (
					3 * radiusV + 3 * range.start + 1) + 4 * (radiusH / 4 + 1);
			float* sptrb = (float*) temp->ptr<float> (
					3 * radiusV + 3 * range.start + 2) + 4 * (radiusH / 4 + 1);
			float* gptr = (float*) guide->ptr<float> (radiusV + range.start)
					+ 4 * (radiusH / 4 + 1);
			float* dptr = dest->ptr<float> (range.start);
			for (i = range.start; i != range.end; i++, gptr += gstep, sptrr
					+= sstep, sptrg += sstep, sptrb += sstep, dptr += dstep) {
				j = 0;

				for (; j < size.width; j++) {
					const float* sptrrj = sptrr + j;
					const float* sptrgj = sptrg + j;
					const float* sptrbj = sptrb + j;
					const float* gptrj = gptr + j;

					const float r0 = gptrj[0];
					float sum_r = 0.0f, sum_b = 0.0f, sum_g = 0.0f;
					float wsum = 0.0f;
					for (k = 0; k < maxk; k++) {
						const float r = gptrj[space_guide_ofs[k]];
						float w = space_weight[k] * color_weight[cvRound(
								std::abs(r - r0))];
						sum_b += sptrrj[space_ofs[k]] * w;
						sum_g += sptrgj[space_ofs[k]] * w;
						sum_r += sptrbj[space_ofs[k]] * w;
						wsum += w;
					}
					wsum = 1.f / wsum;
					dptr[3 * j] = sum_b * wsum;
					dptr[3 * j + 1] = sum_g * wsum;
					dptr[3 * j + 2] = sum_r * wsum;
				}
			}
		}
	}
private:
	const Mat *temp;
	Mat *dest;
	const Mat* guide;
	int radiusH, radiusV, maxk, *space_ofs, *space_guide_ofs;
	float *space_weight, *color_weight;
};

class JointBilateralFilter {
public:
	static void jointBilateralFilter_32f(const Mat& src, const Mat& guide,
			Mat& dst, Size kernelSize, double sigma_color, double sigma_space,
			int borderType);
	static void jointBilateralFilterSP_32f(const Mat& src, const Mat& guide,
			Mat& dst, Size kernelSize, double sigma_color, double sigma_space,
			int borderType);
	static void jointBilateralFilter(const Mat& src, const Mat& guide,
			Mat& dst, Size kernelSize, double sigma_color, double sigma_space,
			int method = BILATERAL_NORMAL,
			int borderType = cv::BORDER_REPLICATE);
};

#endif /* JOINTBILATERALFILTER_H_ */

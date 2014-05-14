
#include <iostream>

#include "Time.h"
#include "JointBilateralFilter.h"

using namespace std;
using namespace cv;

typedef JointBilateralFilter JBF;

////////////////////////////////////////

template<class T>
static void fillOcclusionInv_(Mat& src, const T invalidvalue, const T minval) {
	int bb = 1;
	const int MAX_LENGTH = (int) (src.cols * 0.3);

	for (int j = bb; j < src.rows - bb; j++) {
		T* s = src.ptr<T> (j);

		s[0] = minval;
		s[src.cols - 1] = minval;
		for (int i = 1; i < src.cols - 1; i++) {
			if (s[i] <= invalidvalue) {
				int t = i;
				do {
					t++;
					if (t > src.cols - 2)
						break;
				} while (s[t] <= invalidvalue);

				const T dd = max(s[i - 1], s[t]);
				if (t - i > MAX_LENGTH) {
					for (; i < t; i++) {
						s[i] = invalidvalue;
					}
				} else {
					for (; i < t; i++) {
						s[i] = dd;
					}
				}
			}
		}
		s[0] = s[1];
		s[src.cols - 1] = s[src.cols - 2];
	}

	T* s1 = src.ptr<T> (0);
	T* s2 = src.ptr<T> (1);
	T* s3 = src.ptr<T> (src.rows - 2);
	T* s4 = src.ptr<T> (src.rows - 1);
	for (int i = 0; i < src.cols; i++) {
		s1[i] = s2[i];
		s4[i] = s3[i];
	}
}

void fillOcclusionDepth(Mat& src, int invalidvalue) {
	if (src.type() == CV_16U) {
		fillOcclusionInv_<unsigned short> (src, (unsigned short) invalidvalue,
				0);
	}
}

void depth162depth8Color(Mat& src, Mat& dest, double minv, double maxv)
{
	Mat depthmap8u;
	src-=(short)minv;
	src.convertTo(depthmap8u,CV_8U,255.0/(maxv-minv));
	applyColorMap(depthmap8u,dest,2);
}

int main(int argc, char** argv) {

	string srcimagedir = "./dataset/kinect/meeting_small_1_1.png";
	string srcdepthdir = "./dataset/kinect/meeting_small_1_1_depth.png";

	Mat srcImage = imread(srcimagedir);

	Mat srcDepth = imread(srcdepthdir, -1);
	imshow("INI_DEPTH", srcDepth);

	Mat filledDepth = srcDepth.clone();
	fillOcclusionDepth(filledDepth, 0);
	Mat tp;
	transpose(filledDepth, tp);
	fillOcclusionDepth(tp, 0);
	transpose(tp, filledDepth);

	Mat depthout, depthShow;
	imshow("IM_OCLUSION", filledDepth);

	Mat filledDepthf;
	filledDepth.convertTo(filledDepthf, CV_32F);

	///////////////////////////
	string wname = "PARAM";
	namedWindow(wname);
	int alpha = 0; createTrackbar("alpha",wname,&alpha,100);
	int sw = 0; createTrackbar("sw",wname,&sw,2);
	int r = 3; createTrackbar("r",wname,&r,20);
	int sigs = 30; createTrackbar("sig_s",wname,&sigs,200);
	int sigc = 50; createTrackbar("sig_c",wname,&sigc,255);
	int sigc2 = 50; createTrackbar("sig_c2",wname,&sigc2,255);
	int pr = 2; createTrackbar("pr",wname,&pr,20);

	Time t;

	int key=0;
	while(key!='q'){

		double ss = sigs / 10.0;
		double sc = sigc / 10.0;
		int d = 2 * r + 1;

		if (sw == 0) {
			t.setMessage("Gaussian ");
			t.start();
			Mat filteredDepth;
			GaussianBlur(filledDepth, filteredDepth, Size(d, d), ss);
			filteredDepth.convertTo(depthout, CV_16U);
			t.stop();
		} else if (sw == 1) {
			t.setMessage("Bilateral ");
			Mat filteredDepthf = Mat::ones(srcDepth.size(), CV_32F);
			bilateralFilter(filledDepthf, filteredDepthf, d, sc * 10, ss);
			filteredDepthf.convertTo(depthout, CV_16U);
			t.stop();
		} else if (sw == 2) {
			t.setMessage("Joint ");
			Mat filteredDepthf = Mat::ones(srcDepth.size(), CV_32F);
			Mat srcImageGray;
			cvtColor(srcImage,srcImageGray,CV_BGR2GRAY);
			Mat srcImagef;
			srcImageGray.convertTo(srcImagef,CV_32F);
			JBF::jointBilateralFilter(filledDepthf, srcImagef, filteredDepthf, Size(d, d), sc, ss, 0);
			filteredDepthf.convertTo(depthout, CV_16U);
			t.stop();
		}

		double minv, maxv;
		minMaxLoc(filledDepth, &minv, &maxv);
		depth162depth8Color(depthout,depthShow,minv,maxv);

		addWeighted(srcImage,alpha/100.0,depthShow,1.0-alpha/100.0,0.0,depthShow);

		imshow("RES",depthShow);
		key = waitKey(1);
	}

	return 0;
}

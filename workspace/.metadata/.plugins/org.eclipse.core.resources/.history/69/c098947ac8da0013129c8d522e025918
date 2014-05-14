#include "OpOthers.h"

#include <iostream>

using namespace cv;
using namespace std;

void OpOthers::show_matrix(Mat &M) {

	for (int y = 0; y < M.rows; y++) {
		const double* My = M.ptr<double> (y);
		for (int x = 0; x < M.cols; x++) {
			cout << "M(" << x << "," << y << ") = " << My[x] << endl;
		}
	}

}

int OpOthers::comprobarGamut(int val) {
	if (val < 0)
		val = 0;
	if (val > 255)
		val = 255;

	return val;
}

void OpOthers::traza_rectificationLines(Mat &imgU_L, Mat &imgU_R) {
	for (int j = 0; j < imgU_L.rows; j += 70) {
		line(imgU_L, cvPoint(0, j), cvPoint(imgU_L.cols, j), CV_RGB(0,255,0));
		line(imgU_R, cvPoint(0, j), cvPoint(imgU_L.cols, j), CV_RGB(0,255,0));

		line(imgU_L, cvPoint(j, 0), cvPoint(j, imgU_L.rows), CV_RGB(0,255,0));
		line(imgU_R, cvPoint(j, 0), cvPoint(j, imgU_R.rows), CV_RGB(0,255,0));

	}
}

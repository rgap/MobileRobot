#include "DepthSensing.h"
#include <iostream>
#include "../globals.h"

using namespace cv;
using namespace std;

////////////// SEGMENTACION MOMENTOS - CLASIFICACION   Y SELECCION LATA MAS CERCANA

DepthSensing::DepthSensing() {
}

void DepthSensing::initializePointCloud(Size &szFrame) {
	pointCloud = new PCType(szFrame);
	//PointCloudGrayscale, PointCloudRGB
}

void DepthSensing::visualizePointCloud() {
	pointCloud->visualizeCloud();
}

// OBSTACULOS

void DepthSensing::cloudDownsampling() {
	//pointCloud->fillCloud(imgColor, world, imgMaxDepth, maxDepthDetected);

	//voxelDownsampling.downSample(pointCloud->cloud);
}

void DepthSensing::segmentObstacles(Mat& imgColor, Mat& world,
		Mat& notObstacleBin, Mat &imgMaxDepth, Mat& imgObstaclesBin,
		Mat& imgPlane, float maxDistObstacle, float maxHeightFloor,
		float distThreshold, float epsAngle, int axisVal) {

	pointCloud->fillCloud(imgColor, world, imgMaxDepth, maxDepthDetected);
	//pointCloud->dumbFill();

	sac_method.applyRANSAC(pointCloud->cloud, distThreshold, epsAngle, axisVal);

	int i, j, k = 0;
	for (i = 0; i < world.rows; ++i) {
		uchar*imgMaxDepth_pt = imgMaxDepth.ptr<uchar> (i);
		uchar*notObstacleBin_pt = notObstacleBin.ptr<uchar> (i);
		uchar*imgObstaclesBin_pt = imgObstaclesBin.ptr<uchar> (i);
		uchar*imgPlane_pt = imgPlane.ptr<uchar> (i);
		Vec3f *world_ptr = world.ptr<Vec3f> (i);
		for (j = 0; j < world.cols; ++j) {
			if (sac_method.inPlane[k])
				imgPlane_pt[j] = 255;
			else
				imgPlane_pt[j] = 0;

			if (imgMaxDepth_pt[j] && !notObstacleBin_pt[j]
					&& !sac_method.inPlane[k]) {
				if (world_ptr[j][2] <= maxDistObstacle && i >= (world.rows
						- maxHeightFloor)) {
					imgObstaclesBin_pt[j] = 255;
				} else {
					imgObstaclesBin_pt[j] = 0;
				}
			}
			k++;
		}
	}
	delete(sac_method.inPlane);
}

void DepthSensing::depthThresholding(Mat& world, Mat&imgMaxDepth,
		float maxDepth) {
	int i, j;
	maxDepthDetected = -1;
	for (i = 0; i < world.rows; ++i) {
		uchar*imgMaxDepth_pt = imgMaxDepth.ptr<uchar> (i);
		Vec3f *world_ptr = world.ptr<Vec3f> (i);
		for (j = 0; j < world.cols; ++j) {
			if (world_ptr[j][2] > maxDepthDetected)
				maxDepthDetected = world_ptr[j][2];
			if (world_ptr[j][2] <= maxDepth && world_ptr[j][2] > 0) {
				imgMaxDepth_pt[j] = 255;
			} else {
				imgMaxDepth_pt[j] = 0;
				world_ptr[j][2] = MAX_DEPTH;
			}
		}
	}
}

void DepthSensing::colorizeDisparity(const Mat& gray, Mat& rgb, double maxDisp,
		float S, float V) {
	if (maxDisp <= 0) {
		maxDisp = 0;
		minMaxLoc(gray, 0, &maxDisp);
	}
	rgb.create(gray.size(), CV_8UC3);
	rgb = Scalar::all(0);
	if (maxDisp < 1)
		return;
	int y, x;
	uchar d, b, g, r;
	unsigned int H, hi;
	float f, p, q, t;
	Point3f res;
	for (y = 0; y < gray.rows; y++) {
		for (x = 0; x < gray.cols; x++) {
			d = gray.at<uchar> (y, x);
			H = ((uchar) maxDisp - d) * 240 / (uchar) maxDisp;
			hi = (H / 60) % 6;
			f = H / 60.f - H / 60;
			p = V * (1 - S);
			q = V * (1 - f * S);
			t = V * (1 - (1 - f) * S);
			if (hi == 0) //R = V,        G = t,        B = p
				res = Point3f(p, t, V);
			if (hi == 1) // R = q,        G = V,        B = p
				res = Point3f(p, V, q);
			if (hi == 2) // R = p,        G = V,        B = t
				res = Point3f(t, V, p);
			if (hi == 3) // R = p,        G = q,        B = V
				res = Point3f(V, q, p);
			if (hi == 4) // R = t,        G = p,        B = V
				res = Point3f(V, p, t);
			if (hi == 5) // R = V,        G = p,        B = q
				res = Point3f(q, p, V);
			b = (uchar) (std::max(0.f, std::min(res.x, 1.f)) * 255.f);
			g = (uchar) (std::max(0.f, std::min(res.y, 1.f)) * 255.f);
			r = (uchar) (std::max(0.f, std::min(res.z, 1.f)) * 255.f);
			rgb.at<Point3_<uchar> > (y, x) = Point3_<uchar> (b, g, r);
		}
	}
}

float DepthSensing::getMaxDepth(VideoCapture& capture) {
	const int minDistance = 400; // mm
	float baseline =
			(float) capture.get(CV_CAP_OPENNI_DEPTH_GENERATOR_BASELINE); // mm
	float focalLength = (float) capture.get(
			CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH); // pixels
	return baseline * focalLength / minDistance;
}

void DepthSensing::fillOcclusion(Mat& world, float invalidvalue, float minval) {
	int bb = 1;
	const int MAX_LENGTH = (int) (world.cols * 0.3);
	int i, j;
	for (j = 1; j < world.rows - bb; ++j) {
		Vec3f *s = world.ptr<Vec3f> (j);

		s[0][2] = minval;
		s[world.cols - 1][2] = minval;
		for (i = 1; i < world.cols - 1; ++i) {

			if (s[i][2] <= invalidvalue) {
				int t = i;
				do {
					t++;
					if (t > world.cols - 2)
						break;
				} while (s[t][2] <= invalidvalue);

				float dd = max(s[i - 1][2], s[t][2]);
				if (t - i > MAX_LENGTH) {
					for (; i < t; i++) {
						s[i][2] = invalidvalue;
					}
				} else {
					for (; i < t; i++) {
						s[i][2] = dd;
					}
				}
			}
		}
		s[0][2] = s[1][2];
		s[world.cols - 1][2] = s[world.cols - 2][2];
	}

	Vec3f* s1 = world.ptr<Vec3f> (0);
	Vec3f* s2 = world.ptr<Vec3f> (1);
	Vec3f* s3 = world.ptr<Vec3f> (world.rows - 2);
	Vec3f* s4 = world.ptr<Vec3f> (world.rows - 1);
	for (int i = 0; i < world.cols; i++) {
		s1[i][2] = s2[i][2];
		s4[i][2] = s3[i][2];
	}
}

void DepthSensing::dispMapRefinement(Mat& world) {
	/*
	 int r, c;
	 float promDistance, promDisparidad;
	 for (r = 0; r < disparityMap.rows; ++r) {
	 world.at<Vec3f> (r, c)[2] = promDistance;
	 }
	 */
}

DepthSensing::~DepthSensing() {
}

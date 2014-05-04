#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <SerialStream.h>
#include <iostream>

#include "globals.h"
#include "typedefs.h"

#include "Capture/DeviceCapture.h"
#include "RobotPlanning/RobotFSM.h"
#include "RobotVision.h"
#include "ModelParameters.h"

using namespace cv;
using namespace std;

//////////////////////////////// OPTIONS
#define COLLECT_CANS 1
#define SEND_SIGNALS 1
#define MOVE_KINECT 1

//////////////////////////////// GUI
ModelParameters* P = new ModelParameters();

/*
 //setMouseCallback("imgMaxDepth", onMouseDistance, 0);
 void onMouseDistance(int event, int x, int y, int flags, void*) {
 if (event == CV_EVENT_LBUTTONUP){
 Vec3f *s = vision.world.ptr<Vec3f> (y);
 cout << s[x][0] << " " << s[x][1] << " " << s[x][2] << endl;
 }
 }
 */

void initCallbacks() {
	namedWindow("PARAMS_CAN");

	createTrackbar("sigma", "PARAMS_CAN", &P->sigma, 255, NULL, NULL);
	createTrackbar("alpha", "PARAMS_CAN", &P->alpha, 255, NULL, NULL);
	createTrackbar("beta", "PARAMS_CAN", &P->beta, 255, NULL, NULL);
	createTrackbar("minH_can", "PARAMS_CAN", &P->minH_can, 255, NULL, NULL);
	createTrackbar("maxH_can", "PARAMS_CAN", &P->maxH_can, 255, NULL, NULL);
	createTrackbar("minS_can", "PARAMS_CAN", &P->minS_can, 255, NULL, NULL);
	createTrackbar("maxS_can", "PARAMS_CAN", &P->maxS_can, 255, NULL, NULL);
	createTrackbar("minV_can", "PARAMS_CAN", &P->minV_can, 255, NULL, NULL);
	createTrackbar("maxV_can", "PARAMS_CAN", &P->maxV_can, 255, NULL, NULL);
	createTrackbar("blurSize", "PARAMS_CAN", &P->blurSize, 51, NULL, NULL);
	createTrackbar("minArea", "PARAMS_CAN", &P->minArea, 50000, NULL, NULL);
	createTrackbar("maxArea", "PARAMS_CAN", &P->maxArea, 50000, NULL, NULL);
	createTrackbar("minApCuadrado", "PARAMS_CAN", &P->minApCuadrado_pos, 10,
			NULL, NULL);
	createTrackbar("maxApCuadrado", "PARAMS_CAN", &P->maxApCuadrado_pos, 10,
			NULL, NULL);

	namedWindow("PARAMS_DEPTH");
	createTrackbar("maxDepth", "PARAMS_DEPTH", &P->max_depth_pos, 20, NULL,
			NULL);
	createTrackbar("maxDistObstacle", "PARAMS_DEPTH", &P->maxDistObstacle_pos,
			30, NULL, NULL);
	createTrackbar("maxHeightFloor", "PARAMS_DEPTH", &P->maxHeightFloor_pos,
			480, NULL, NULL);
	createTrackbar("distThreshold", "PARAMS_DEPTH", &P->distThreshold_pos, 480,
			NULL, NULL);
	createTrackbar("axisVal", "PARAMS_DEPTH", &P->axisVal, 2, NULL, NULL);
	createTrackbar("epsAngle", "PARAMS_DEPTH", &P->epsAngle_pos, 3600, NULL,
			NULL);

	namedWindow("PARAMS_WATER");
	createTrackbar("minH_water", "PARAMS_WATER", &P->minH_water, 255, NULL,
			NULL);
	createTrackbar("maxH_water", "PARAMS_WATER", &P->maxH_water, 255, NULL,
			NULL);
	createTrackbar("minS_water", "PARAMS_WATER", &P->minS_water, 255, NULL,
			NULL);
	createTrackbar("maxS_water", "PARAMS_WATER", &P->maxS_water, 255, NULL,
			NULL);
	createTrackbar("minV_water", "PARAMS_WATER", &P->minV_water, 255, NULL,
			NULL);
	createTrackbar("maxV_water", "PARAMS_WATER", &P->maxV_water, 255, NULL,
			NULL);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


void frameLoop() {

	////////////////////////// HW
	//DeviceCapture device(2, "/home/rgap/REPOSITORIES/BeachCleaner/CloudTesting/dataset/kinect/meeting_small_1_1.png","/home/rgap/REPOSITORIES/BeachCleaner/CloudTesting/dataset/kinect/meeting_small_1_1_depth.png");
	DeviceCapture device(1);

	RobotVision vision;
	RobotFSM robot;

	vision.initializeVision(device.szFrame);
	vision.setParameters(P);
	initCallbacks();

	int k, saveTesting = 0;
	while (1) {

		device.verifyState();
		P->updateParameters();

		vision.imgBGR = device.getBGRFrame();
		imshow("imgBGR", vision.imgBGR);
		if (saveTesting)
			imwrite("TESTS/1_imgBGR.png", vision.imgBGR);

		vision.applyGaussBlur(vision.imgBGR, vision.imgBGR_Blur);
		imshow("imgBGR_Blur", vision.imgBGR_Blur);
		if (saveTesting)
			imwrite("TESTS/1_imgBGR_Blur.png", vision.imgBGR_Blur);

		vision.applySSR(vision.imgBGR_Blur, vision.imgBGR_Enh);
		imshow("imgBGR_Enh", vision.imgBGR_Enh);
		if (saveTesting)
			imwrite("TESTS/1_imgBGR_Enh.png", vision.imgBGR_Enh);

		vision.applyColorSegmentation_HSV(vision.imgBGR_Enh,
				vision.imgSegColor_Can);
		imshow("can_HSV", vision.imgSegColor_Can);
		if (saveTesting)
			imwrite("TESTS/1_imgSegColor_Can.png", vision.imgSegColor_Can);

		{
			vision.world = device.getDepthFrame();

			vision.getColorizeDisparity(device.getDisparityMap());
			imshow("dispColor", vision.disparityColorized);
			if (saveTesting)
				imwrite("TESTS/1_disparityColorized.png", vision.disparityColorized);

			// Occlusion Filling
			//vision.applyFillOcclusion(vision.world);

			////////////////////////////////////// RANGO DE VISION
			vision.applyDepthThresholding(vision.world, vision.imgMaxDepth);
			imshow("imgMaxDepth", vision.imgMaxDepth);
			if (saveTesting)
				imwrite("TESTS/1_imgMaxDepth.png", vision.imgMaxDepth);

			vision.applyShapeSegmentation(vision.imgSegColor_Can,
					vision.imgSegShape_Can);
			imshow("imgSegShape_Can", vision.imgSegShape_Can);
			imshow("imgBGR_cans_detected", vision.imgBGR);

			if (saveTesting)
				imwrite("TESTS/1_imgSegShape_Can.png", vision.imgSegShape_Can);
			if (saveTesting)
				imwrite("TESTS/1_imgBGR_cans_detected.png", vision.imgBGR);

			//vision.applyCloudDownsampling();

			////////////////////////////////////// OBTACLE RECOGNITION
			vision.applyObstacleRecognition(vision.imgBGR_Blur, vision.world,
					vision.imgSegShape_Can, vision.imgObstaclesBin);
			imshow("imgObstaclesBin", vision.imgObstaclesBin);
			imshow("imgPlane", vision.imgPlane);

			if (saveTesting)
				imwrite("TESTS/1_imgObstaclesBin.png", vision.imgObstaclesBin);
			if (saveTesting)
				imwrite("TESTS/1_imgPlane.png", vision.imgPlane);
		}
		{
			//robot.planWithEnvironment(vision.getPointNearestCan(),
			//		vision.imgObstaclesBin);
		}

		if (saveTesting)
			saveTesting = 0;

		k = cv::waitKey(1);
		if (k == ' ') {
			saveTesting = 1;
		}
	}
}

int main() {
	frameLoop();
}

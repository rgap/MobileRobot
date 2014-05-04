/*
 * RobotFSM.cpp
 *
 *  Created on: 02/04/2014
 *      Author: rgap
 */

#include "RobotFSM.h"

using namespace cv;
using namespace std;

RobotFSM::RobotFSM() {
}

void RobotFSM::planWithEnvironment(Point3f *pointNearestCan,
		Mat &imgObstaclesBin) {

	int frameWidth = imgObstaclesBin.cols, rangoGiro = 150, i;
	imgObstaclesBin_temp = imgObstaclesBin.clone();
	findContours(imgObstaclesBin_temp, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	// Aprox Poly DP
	vector<Rect> boundRect(contours.size());
	for (i = 0; i < contours.size(); i++) {
		boundRect[i] = boundingRect(Mat(contours[i]));
	}

	for (i = 0; i < contours.size(); i++) {
		if (abs(boundRect[i].tl().x - frameWidth / 2) > abs(
				boundRect[i].br().x - frameWidth / 2)) { // LADO IZQUIERDO

			if ((frameWidth / 2 - boundRect[i].br().x) < rangoGiro) {
				cout << "TURN RIGHT & GO BACKWARD" << endl; //TURN RIGHT
				//cout << "OBSTACLE DETECTED" << endl;
			}
		} else { // LADO DERECHO
			if ((boundRect[i].tl().x - frameWidth / 2) < rangoGiro) {
				cout << "TURN LEFT & GO BACKWARD" << endl; //TURN LEFT
				//cout << "OBSTACLE DETECTED" << endl;
			}
		}
	}

}

RobotFSM::~RobotFSM() {
	// TODO Auto-generated destructor stub
}

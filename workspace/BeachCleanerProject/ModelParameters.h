/*
 * ModelParameters.h
 *
 *  Created on: 20/10/2013
 *      Author: rgap
 */

#ifndef MODELPARAMETERS_H_
#define MODELPARAMETERS_H_

class ModelParameters {
public:

	float verticalPositionKinect;
	//////////////////////////////// COLLECT CAN
	int rangeCollectCan;
	float distanceCollectCan;
	float distanceMinObstacle;

	////////////////////
	int sigma;
	int alpha;
	int beta;

	///////////////////////////////////////////
	int minH_can, maxH_can, minS_can, maxS_can, minV_can, maxV_can;
	int minH_water, maxH_water, minS_water, maxS_water, minV_water, maxV_water;

	int blurSize;
	int minArea;
	int maxArea;
	float minApCuadrado;
	float maxApCuadrado;

	int max_depth_pos; //2.0 m
	int maxDistObstacle_pos;
	int maxHeightFloor_pos;
	int distThreshold_pos;
	int epsAngle_pos;
	int minApCuadrado_pos;
	int maxApCuadrado_pos;

	int axisVal;

	float min_apCuadrado;
	float max_depth;
	float maxDistObstacle;
	float maxHeightFloor;
	float distThreshold;
	float epsAngle;

	////////////
	ModelParameters() {
		verticalPositionKinect = 0.42; //m
		rangeCollectCan = 12; //px
		distanceCollectCan = 0.30; //m
		distanceMinObstacle = 0.8; //m

		sigma = 50;
		alpha = 140;
		beta = 120;

		axisVal = 1;
		epsAngle_pos = 2450;

		////////////////////////////
		minH_can = 0, maxH_can = 0, minS_can = 0, maxS_can = 0, minV_can = 0, maxV_can
				= 1;
		minH_water = 91, maxH_water = 164, minS_water = 85, maxS_water = 193, minV_water
				= 125, maxV_water = 254;

		blurSize = 9; //px
		minArea = 4900; //px
		maxArea = 23440; //px
		max_depth_pos = 4; //2.0 m
		maxDistObstacle_pos = 16;
		maxHeightFloor_pos = 480; //Height = 480
		distThreshold_pos = 170;
		minApCuadrado_pos = 7;
		maxApCuadrado_pos = 10;
	}

	void updateParameters() {
		if (blurSize % 2 == 0)
			blurSize++;
		maxDistObstacle = maxDistObstacle_pos * 1.0 / 10;
		maxHeightFloor = (int) maxHeightFloor_pos;
		distThreshold = distThreshold_pos * 1.0 / 10;
		max_depth = max_depth_pos * 0.5;
		// ransac
		epsAngle = epsAngle_pos * 1.0 / 10;
		minApCuadrado = minApCuadrado_pos * 1.0 / 10;
		maxApCuadrado = maxApCuadrado_pos * 1.0 / 10;
	}
};

#endif /* MODELPARAMETERS_H_ */


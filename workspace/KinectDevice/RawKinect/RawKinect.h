/*
 * RawKinect.h
 *
 *  Created on: 26/11/2013
 *      Author: rgap
 */

#ifndef RAWKINECT_H_
#define RAWKINECT_H_

#include <XnUSB.h>
#include <cstdio>

class RawKinect {
public:
	enum {
		MaxDevs = 16
	};

	enum LedColor {
		LED_OFF = 0, LED_GREEN = 1, LED_RED = 2, LED_YELLOW = 3, //(actually orange)
		LED_BLINK_YELLOW = 4, //(actually orange)
		LED_BLINK_GREEN = 5,
		LED_BLINK_RED_YELLOW = 6
	//(actually red/orange)
	};

	int angleKinect;

public:
	RawKinect();
	virtual ~RawKinect();

	/**
	 * Open device.
	 * @return true if succeeded, false - overwise
	 */
	bool Open();

	/**
	 * Close device.
	 */
	void Close();

	/**
	 * Move motor up or down to specified angle value.
	 * @param angle angle value
	 * @return true if succeeded, false - overwise
	 */
	bool Move(int angle, int onMove);
	bool MoveInc(int angleInc);

	/**
	 * Control kinect led
	 * @param color led control value
	 */
	bool setLed(LedColor color);

	bool getAccelerometer(float &x, float &y, float &z);

private:
	XN_USB_DEV_HANDLE m_devs[MaxDevs];
	XnUInt32 m_num;
	bool m_isOpen;
};

#endif /* RAWKINECT_H_ */

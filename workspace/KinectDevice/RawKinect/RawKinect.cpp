/*
 * RawKinect.cpp
 *
 *  Created on: 17/10/2013
 *      Author: rgap
 */

#include "RawKinect.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

RawKinect::RawKinect() {
	m_isOpen = false;
	angleKinect = 0;
}

RawKinect::~RawKinect() {
	Close();
}

bool RawKinect::Open() {
	const XnUSBConnectionString *paths;
	XnUInt32 count;
	XnStatus res;
	// Init OpenNI USB
	res = xnUSBInit();
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBInit failed");
		return false;
	}

	// Open all "Kinect motor" USB devices
	res = xnUSBEnumerateDevices(0x045E /* VendorID */, 0x02B0 /*ProductID*/,
			&paths, &count);
	if (res != XN_STATUS_OK) {
		xnPrintError(res, "xnUSBEnumerateDevices failed");
		return false;
	}

	// Open devices
	for (XnUInt32 index = 0; index < count; ++index) {
		res = xnUSBOpenDeviceByPath(paths[index], &m_devs[index]);
		if (res != XN_STATUS_OK) {
			xnPrintError(res, "xnUSBOpenDeviceByPath failed");
			return false;
		}
	}
	m_num = count;
	XnUChar buf[1]; // output buffer
	// Init motors
	for (XnUInt32 index = 0; index < m_num; ++index) {
		res = xnUSBSendControl(m_devs[index], (XnUSBControlType) 0xc0, 0x10,
				0x00, 0x00, buf, sizeof(buf), 0);
		if (res != XN_STATUS_OK) {
			xnPrintError(res, "xnUSBSendControl failed");
			Close();
			return false;
		}
		res = xnUSBSendControl(m_devs[index], XN_USB_CONTROL_TYPE_VENDOR, 0x06,
				0x01, 0x00, NULL, 0, 0);
		if (res != XN_STATUS_OK) {
			xnPrintError(res, "xnUSBSendControl failed");
			Close();
			return false;
		}
	}
	m_isOpen = true;
	return true;
}

void RawKinect::Close() {
	if (m_isOpen) {
		for (XnUInt32 index = 0; index < m_num; ++index)
			xnUSBCloseDevice(m_devs[index]);
		m_isOpen = false;
	}
}

bool RawKinect::MoveInc(int angleInc) {
	cout << "angle: " << angleKinect << " -> ";
	angleKinect += angleInc;
	bool res = Move(angleKinect, 0);
	return res;
}

bool RawKinect::Move(int angle, int onMove) {
	if (onMove) {
		if (angle == angleKinect)
			return true;
		cout << "angle: " << angleKinect << " -> ";
	}
	angleKinect = angle;
	cout << angleKinect << endl;
	XnStatus res;
	for (XnUInt32 index = 0; index < m_num; ++index) {
		res = xnUSBSendControl(m_devs[index], XN_USB_CONTROL_TYPE_VENDOR, 0x31,
				angle, 0x00, NULL, 0, 0);
		//cout<<"resMotor = "<<res<<endl;
		if (res != XN_STATUS_OK) {
			xnPrintError(res, "xnUSBSendControl failed");
			return false;
		}
	}
	sleep(1);
	return true;
}

bool RawKinect::setLed(RawKinect::LedColor color) {
	XnStatus res;
	// Send move control requests
	for (XnUInt32 index = 0; index < m_num; ++index) {
		res = xnUSBSendControl(m_devs[index], XN_USB_CONTROL_TYPE_VENDOR, 0x06,
				color, 0x00, NULL, 0, 0);
		if (res != XN_STATUS_OK) {
			xnPrintError(res, "xnUSBSendControl failed");
			return false;
		}
	}
	return true;
}

bool RawKinect::getAccelerometer(float &x, float &y, float &z) {
	XnStatus res;
	XnUInt32 nBufferSize = 10;
	XnUChar * pBuffer = new XnUChar[nBufferSize];
	XnUInt32 pnBytesReceived;
	int16_t ux, uy, uz;
	// Send move control requests
	for (XnUInt32 index = 0; index < m_num; ++index) {
		res = xnUSBReceiveControl(m_devs[index], XN_USB_CONTROL_TYPE_VENDOR,
				0x32, 0x00, 0x00, pBuffer, nBufferSize, &pnBytesReceived, 0);
		if (res != XN_STATUS_OK) {
			xnPrintError(res,
					"[RawKinect]::getAccelerometer() : xnUSBReceiveControl failed");
			return false;
		}
		ux = (int16_t) (((short) pBuffer[2] << 8) | pBuffer[3]);
		uy = (int16_t) (((short) pBuffer[4] << 8) | pBuffer[5]);
		uz = (int16_t) (((short) pBuffer[6] << 8) | pBuffer[7]);
		// recalculate gravitation vector
		x = 1.0 * ux / 819;
		y = 1.0 * uy / 819;
		z = 1.0 * uz / 819;
	}
	return true;
}

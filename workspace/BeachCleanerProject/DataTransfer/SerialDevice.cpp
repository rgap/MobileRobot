/*
 * SerialControlSignals.cpp
 *
 *  Created on: 30/12/2013
 *      Author: rgap
 */

#include "SerialDevice.h"
#include <iostream>
#include <fstream>

SerialDevice::SerialDevice() {
}

void SerialDevice::setDevice(std::string devicePath_) {
	devicePath = devicePath_;
}

void SerialDevice::initializeSerialDevice() {
	serialStream = new LibSerial::SerialStream(devicePath, std::ios_base::in | std::ios_base::out);
	serialStream->SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
	serialStream->SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
	std::cout << "Device Opened : " << serialStream->IsOpen() << std::endl;
}

SerialDevice::~SerialDevice() {
}

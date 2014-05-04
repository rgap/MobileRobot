/*
 * SerialControlSignals.cpp
 *
 *  Created on: 30/12/2013
 *      Author: rgap
 */

#include "SerialDevice.h"
#include <iostream>
#include <fstream>

SerialDevice::SerialDevice(std::string path_device) {
	path_device_ = path_device;
}

void SerialDevice::initializeSerialDevice() {
	serialStream = new LibSerial::SerialStream(path_device_, std::ios_base::in | std::ios_base::out);
	serialStream->SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
	serialStream->SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
	std::cout << "Open : " << serialStream->IsOpen() << std::endl;
}

SerialDevice::~SerialDevice() {
}

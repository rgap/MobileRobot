#include "Time.h"

void Time::start() {
	timeTemp = getTickCount();
}

void Time::restart() {
	start();
}

void Time::show() {
	getTime();
	switch (timeMode) {
	case TIME_NSEC:
		cout << message << ": " << elapsedTime << " nsec" << endl;
		break;
	case TIME_SEC:
		cout << message << ": " << elapsedTime << " sec" << endl;
		break;
	case TIME_MIN:
		cout << message << ": " << elapsedTime << " minute" << endl;
		break;
	case TIME_HOUR:
		cout << message << ": " << elapsedTime << " hour" << endl;
		break;
	case TIME_MSEC:
	default:
		cout << message << ": " << elapsedTime << " msec" << endl;
		break;
	}
}

void Time::show(string message) {
	getTime();
	switch (timeMode) {
	case TIME_NSEC:
		cout << message << ": " << elapsedTime << " nsec" << endl;
		break;
	case TIME_SEC:
		cout << message << ": " << elapsedTime << " sec" << endl;
		break;
	case TIME_MIN:
		cout << message << ": " << elapsedTime << " minute" << endl;
		break;
	case TIME_HOUR:
		cout << message << ": " << elapsedTime << " hour" << endl;
		break;
	case TIME_MSEC:
	default:
		cout << message << ": " << elapsedTime << " msec" << endl;
		break;
	}
}

double Time::getTime() {
	elapsedTime = (getTickCount() - timeTemp) / (getTickFrequency());
	switch (timeMode) {
	case TIME_NSEC:
		elapsedTime *= 1000000.0;
		break;
	case TIME_SEC:
		elapsedTime *= 1.0;
		break;
	case TIME_MIN:
		elapsedTime /= (60.0);
		break;
	case TIME_HOUR:
		elapsedTime /= (60 * 60);
		break;
	case TIME_MSEC:
	default:
		elapsedTime *= 1000.0;
		break;
	}
	return elapsedTime;
}

void Time::setMessage(string message_) {
	message = message_;
}

void Time::setMode(int timeMode_) {
	timeMode = timeMode_;
}

void Time::stop() {
	getTime();
	show();
}

Time::Time(string message, int timeMode_) {
	timeMode = timeMode_;
}

Time::~Time() {
}

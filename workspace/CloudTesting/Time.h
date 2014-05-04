#ifndef TIME_H_
#define TIME_H_

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

enum {
	TIME_NSEC = 0, TIME_MSEC, TIME_SEC, TIME_MIN, TIME_HOUR
};

class Time {
	int64 timeTemp;
	string message;
	int timeMode;
	double elapsedTime;
	bool showTime;
public:
	void start();
	void stop();
	void setMode(int timeMode_);
	void setMessage(string message);
	void restart();
	double getTime();
	void show();
	void show(string message);

	Time(string message = "time ", int mode = TIME_MSEC);
	~Time();
};

#endif /* TIME_H_ */

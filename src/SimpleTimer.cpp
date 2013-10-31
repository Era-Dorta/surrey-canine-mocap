#include "SimpleTimer.h"

using std::cerr;
using std::endl;

Timer::Timer(char* description) :
			_description(description) {
	start_ticks = 0.0;
	end_ticks = 0.0;
	//Automatically start timing:
	tick();
}

void Timer::tick() {
	start_ticks = (double) cv::getTickCount();
}

float Timer::tock() {
	if (start_ticks > 0) {
		end_ticks = (double) cv::getTickCount();
		time_ms = 1000 * (end_ticks - start_ticks)
				/ ((double) cv::getTickFrequency());
		start_ticks = 0.0;
	} else {
		cerr << "Error: tock before tick" << endl;
		return -1;
	}
	return time_ms;
}

void Timer::print_timing() {
	//cout << _description << " - " << time_ms << " ms" << endl;
	printf("%s - %.1f ms\n", _description, time_ms);
}

void Timer::tock_print() {
	tock();
	print_timing();
}

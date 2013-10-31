#ifndef SIMPLETIMER_H_
#define SIMPLETIMER_H_

#include <iostream>
#include <stdio.h>
#include "opencv2/opencv.hpp"

//A simple wrapper class for the OpenCV timing functionality:
class Timer {
	public:
		Timer(char* description);
		virtual ~Timer() {
		}
		void tick();
		float tock();
		void print_timing();
		void tock_print();
	private:
		char* _description;
		double start_ticks;
		double end_ticks;
		float time_ms;
};

#endif

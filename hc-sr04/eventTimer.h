
#include <Arduino.h>

#ifndef __EVENTTIMER_H
#define __EVENTTIMER_H

class eventTimer {
private: 


	//duration of timer
	uint32_t runTime;
	//recorded start time of timer
	uint32_t startTime;
	//are we running?
	bool isRunning = true;

public:

	eventTimer(void);
	void start(uint32_t timeMS = 1000); 
	bool checkExpired(void);
	void cancel(void);
};
#endif
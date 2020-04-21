#include "eventTimer.h"


eventTimer::eventTimer() {

}


void eventTimer::start(uint32_t rt) {
	//are we running?
	isRunning = true;
	//record start time for future reference (ms)
	startTime = millis();
	//duration we want to run timer for (ms)
	runTime = rt;
}


bool eventTimer::checkExpired() {
	//are we running?
	if (isRunning) { 
		//if so then check if timer has expired
		if (millis() > (runTime+startTime)) { 
			//we stop running since timer has run out
			isRunning = false;
			//tell users timer has expired
			return true; 
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}


void eventTimer::cancel() {
	isRunning = false;
}
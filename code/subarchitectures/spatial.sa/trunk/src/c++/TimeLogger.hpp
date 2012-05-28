#ifndef TIME_LOGGER_HPP
#define TIME_LOGGER_HPP

#include <cast/core/CASTComponent.hpp>

using namespace cast;

class TimeLogger {
public:
	TimeLogger(CASTComponent* _owner, const char *_str, int _line) :
		owner(_owner), str(_str), line(_line) {
		startTime = owner->getCASTTime();
	}
	~TimeLogger() {
		owner->log("Time @%s:%i = %f", str, line, (double) (owner->getCASTTime().s
				- startTime.s) + 1e-6 * (owner->getCASTTime().us - startTime.us));
	}
	CASTComponent* owner;
	cdl::CASTTime startTime;
	const char *str;
	int line;
};
#endif

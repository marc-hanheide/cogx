/** @file Message.cpp
 * 
 * Messages for multithread systems.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Sys/Thread.h>
#include <Golem/Tools/Message.h>
#include <stdio.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

char* golem::vsnprintf(char* begin, const char* end, const char* format, va_list argptr) {
	if (begin >= end)
		return begin;

	const size_t size = end - begin;
	
	#ifdef WIN32
		#pragma warning (push)
		#pragma warning (disable:4996)
		#define vsnprintf _vsnprintf
	#endif
	
	const int inc = ::vsnprintf(begin, size, format, argptr);

	#ifdef WIN32
		#undef vsnprintf
		#pragma warning (pop)
	#endif
	
	if (inc < 0)
		begin += size;
	else if (size > size_t(inc))
		return begin + inc; // with NULL character
	else
		begin += inc;
	
	*begin = '\0'; // always terminate with NULL character

	return begin;
}

char* golem::snprintf(char* begin, const char* end, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	begin = vsnprintf(begin, end, format, argptr);
	va_end(argptr);
	return begin;
}

//------------------------------------------------------------------------------

const SecTmReal Message::TIME_UNDEF = SEC_TM_REAL_INF;

const char *const Message::LEVEL_STR[] = {
	"UNDEF",
	"DEBUG",
	"INFO",
	"NOTICE",
	"WARNING",
	"ERROR",
	"CRIT",
	"ALERT",
	"EMERG",
};

const PerfTimer* Message::_timer = NULL;

//------------------------------------------------------------------------------

Message::Message() : _time(TIME_UNDEF), _thread(THREAD_UNDEF), _level(LEVEL_UNDEF), _code(CODE_UNDEF) {
}

Message::~Message() throw () {
}

Message::Message(const Message& message) {
	*this = message;
}

Message::Message(const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	create(TIME_UNDEF, THREAD_UNDEF, LEVEL_UNDEF, CODE_UNDEF, format, argptr);
	va_end(argptr);
}

Message::Message(Level level, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	create(getCurrentTime(), getCurrentThread(), level, CODE_UNDEF, format, argptr);
	va_end(argptr);
}

Message::Message(Level level, Code code, const char* format, ...) {
	va_list argptr;
	va_start(argptr, format);
	create(getCurrentTime(), getCurrentThread(), level, code, format, argptr);
	va_end(argptr);
}

//------------------------------------------------------------------------------

Message& Message::operator = (const Message& message) {
	_time = message._time;
	_thread = message._thread;
	_level = message._level;
	_code = message._code;
	_str = message._str;
	return *this;
}

void  Message::setTimer(const PerfTimer* timer) {
	if (timer != NULL) _timer = timer;
}

SecTmReal Message::getCurrentTime() {
	return _timer ? _timer->elapsed() : TIME_UNDEF;
}

U32 Message::getCurrentThread() {
	return Thread::getCurrentThreadId();
}

//------------------------------------------------------------------------------

void Message::create(SecTmReal time, U32 thread, Level level, Code code, const char* format, va_list argptr) {
	char buf[BUFSIZ], *begin = buf, *const end = buf + sizeof(buf) - 1;

	_time = time;
	_thread = thread;
	_level = level;
	_code = code;
	
	if (_time != TIME_UNDEF || _thread != THREAD_UNDEF || _level != LEVEL_UNDEF || _code != CODE_UNDEF) {
		if (begin < end)
			*begin++ = '[';
		
		if (_time != TIME_UNDEF)
			begin = golem::snprintf(begin, end, "time=%.6f, ", _time);
		
		if (_thread != THREAD_UNDEF)
			begin = golem::snprintf(begin, end, "thread=%u", _thread);

		if (_level != LEVEL_UNDEF)
			begin = golem::snprintf(begin, end, ", level=%s", LEVEL_STR[_level]);

		if (_code != CODE_UNDEF)
			begin = golem::snprintf(begin, end, ", code=%u", _code);

		if (begin < end)
			*begin++ = ']';
		
		if (begin < end)
			*begin++ = ' ';
	}

	begin = golem::vsnprintf(begin, end, format, argptr);

	_str.assign(buf, begin);
}

//------------------------------------------------------------------------------

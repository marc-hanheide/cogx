/** @file Message.h
 * 
 * Messages for multithreaded systems.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_MESSAGE_H_
#define _GOLEM_TOOLS_MESSAGE_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Sys/Timer.h>
#include <stdarg.h>
#include <string>
#include <exception>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** vsnprintf with pointers */
char* vsnprintf(char* begin, const char* end, const char* format, va_list argptr);

/** snprintf with pointers */
char* snprintf(char* begin, const char* end, const char* format, ...);

//------------------------------------------------------------------------------

/** Golem standard message/exception with information about thread id and time stamp. 
*/
class Message : public std::exception {
public:
	typedef shared_ptr<Message> Ptr;

	/** Undefined message time (TIME_UNDEF is ignored in string) */
	static const SecTmReal TIME_UNDEF;
	
	/** Undefined message thread id (THREAD_UNDEF is ignored in string) */
	static const U32 THREAD_UNDEF = numeric_const<U32>::MAX;

	/** Message importance level (LEVEL_UNDEF is ignored in string) */
	enum Level {
		LEVEL_UNDEF,
		LEVEL_DEBUG,
		LEVEL_INFO,
		LEVEL_NOTICE,
		LEVEL_WARNING,
		LEVEL_ERROR,
		LEVEL_CRIT,
		LEVEL_ALERT,
		LEVEL_EMERG,
	};
	
	/** Level as string */
	static const char *const LEVEL_STR[];

	/** Message code */
	typedef U32 Code;
	
	/** Undefined message code (CODE_UNDEF is ignored in string) */
	static const Code CODE_UNDEF = 0;

private:
	static const PerfTimer* _timer;
	
protected:
	SecTmReal _time;
	U32 _thread;
	Level _level;
	Code _code;
	std::string _str; // can throw during construction
	
public:
	/** Default constructor */
	Message();
	
	/** Destructor may throw */
	virtual ~Message() throw ();
	
	/** Copy constructor */
	Message(const Message& message);
	
	/** Construct a message from formated string without information about time and thread */
	Message(const char* format, ...);
	
	/** Construct a message from formated string with information about time, thread and level */
	Message(Level level, const char* format, ...);
	
	/** Construct a message from formated string with information about time, thread, level and code */
	Message(Level level, Code code, const char* format, ...);

	/**	Assignment operator. */
	Message& operator = (const Message& message);

	/** Message setup called by Message constructor */ 
	void create(SecTmReal time, U32 thread, Level level, Code code, const char* format, va_list argptr);
	
	/** Set timer only if different than NULL */
	static void setTimer(const PerfTimer* timer);
	
	/** Get current time */
	static SecTmReal getCurrentTime();
	/** Get current thread */
	static U32 getCurrentThread();
	
	/** Returns message time stamp */
	SecTmReal time() const {
		return _time;
	}
	/** Returns message thread */
	U32 thread() const {
		return _thread;
	}
	/** Returns message level */
	Level level() const {
		return _level;
	}
	/** Returns message code */
	Code code() const {
		return _code;
	}
	/** Returns message string */
	const std::string& str() const {
		return _str;
	}

	/** Exception class override */
	virtual const char *what() const throw () {
		return _str.c_str();
	}
};

//------------------------------------------------------------------------------

#define MESSAGE_BODY(NAME)\
public:\
NAME()\
{}\
NAME(const NAME& name) {\
	(Message&)*this = name;\
}\
NAME(const char* format, ...) {\
	va_list argptr;\
	va_start(argptr, format);\
	create(TIME_UNDEF, THREAD_UNDEF, LEVEL_UNDEF, CODE_UNDEF, format, argptr);\
	va_end(argptr);\
}\
NAME(Message::Level level, const char* format, ...) {\
	va_list argptr;\
	va_start(argptr, format);\
	create(getCurrentTime(), getCurrentThread(), level, CODE_UNDEF, format, argptr);\
	va_end(argptr);\
}\
NAME(Message::Level level, Message::Code code, const char* format, ...) {\
	va_list argptr;\
	va_start(argptr, format);\
	create(getCurrentTime(), getCurrentThread(), level, code, format, argptr);\
	va_end(argptr);\
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MESSAGE_H_*/

/** @file Timer.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_SYS_TIMER_H_
#define _GOLEM_SYS_TIMER_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Types.h>
#include <Golem/Defs/Constants.h>

#ifdef WIN32
#include <time.h>
#include <Winsock2.h>
#elif LINUX
#include <sys/time.h>
#endif

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Millisecond timer type (integer) */
typedef U32 MSecTmU32;
/** Time constants for millisecond timer type */
const MSecTmU32 MSEC_TM_U32_ZERO = numeric_const<MSecTmU32>::ZERO;
const MSecTmU32 MSEC_TM_U32_ONE = numeric_const<MSecTmU32>::ONE;
const MSecTmU32 MSEC_TM_U32_INF = numeric_const<MSecTmU32>::MAX;
const MSecTmU32 MSEC_TM_U32_MIN = numeric_const<MSecTmU32>::MIN;
const MSecTmU32 MSEC_TM_U32_MAX = numeric_const<MSecTmU32>::MAX;

/** Second timer type (floating point) */
typedef F64 SecTmReal;
/** Time constants for second timer type */
const SecTmReal SEC_TM_REAL_ZERO = numeric_const<SecTmReal>::ZERO;
const SecTmReal SEC_TM_REAL_ONE = numeric_const<SecTmReal>::ONE;
const SecTmReal SEC_TM_REAL_INF = numeric_const<SecTmReal>::MAX;
const SecTmReal SEC_TM_REAL_MIN = numeric_const<SecTmReal>::MIN;
const SecTmReal SEC_TM_REAL_MAX = numeric_const<SecTmReal>::MAX;

/** Conversion SecTmReal to MSecTmU32 */
MSecTmU32 SecTmRealToMSecTmU32(SecTmReal t);
/** Conversion MSecTmU32 to SecTmReal */
SecTmReal MSecTmU32ToSecTmReal(MSecTmU32 t);

//------------------------------------------------------------------------------

/** Standard system timer with millisecond accuracy.
 */
class SysTimer {
private:
#ifdef WIN32
	U32 stamp;
#elif LINUX
	timespec stamp;
#endif

public:
	/** Default constructor initialises and resets the timer.
	*/
	SysTimer();

	~SysTimer();

	/** Resets the timer.
	 */
	void reset();

	/** Calculates the time which elapsed since the last reset of the timer.
	 * 
	 * @return	elapsed time in milliseconds
	 */
	MSecTmU32 elapsed() const;

	/** Suspends the calling thread for <code>interval</code> milliseconds.
	 * 
	 * @param interval	time in miliseconds
	 */
	static void sleep(MSecTmU32 interval);
};

//------------------------------------------------------------------------------

/** Floating-point precision timer.
 * 
 * Timer with microsecond accuracy.
 */
class PerfTimer {
private:
#ifdef WIN32
	LARGE_INTEGER sysFreq, perfStamp;
#elif LINUX
	timespec stamp;
#endif
	
public:
	/** Default constructor initialises and resets the timer.
	*/
	PerfTimer();

	~PerfTimer();
	
	/** Resets the timer.
	 */
	void reset();

	/** Calculates the time which elapsed since the last reset of the timer.
	 * 
	 * @return	elapsed time in seconds
	 */
	SecTmReal elapsed() const;
	
	/** Suspends the calling thread for <code>interval</code> seconds.
	 * 
	 * @param interval	time in seconds
	 */
	static void sleep(SecTmReal interval);
};

//------------------------------------------------------------------------------

#ifdef WIN32
/** Windows implementation of timezone */
struct timezone {
	/** minutes W of Greenwich */
	int tz_minuteswest;
	/** type of dst correction */
	int tz_dsttime;
};

/** Windows implementation of gettimeofday() */
int gettimeofday(timeval *tv, timezone *tz);
#endif // Windows

/** computes difference between two timeval */
timeval timevaldiff(const timeval& tv0, const timeval& tv1);

//------------------------------------------------------------------------------

#ifdef LINUX
/** Normalizes timespec */
void timespecnorm(timespec& ts);
#endif	// Linux

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_TIMER_H_*/

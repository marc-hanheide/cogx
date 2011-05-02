/** @file Timer.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Sys/Timer.h>

#ifdef WIN32
#include <Mmsystem.h>
#elif LINUX
#include <time.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

MSecTmU32 golem::SecTmRealToMSecTmU32(SecTmReal t) {
	return	t <= SEC_TM_REAL_ZERO ? MSEC_TM_U32_ZERO : t >= SecTmReal(MSEC_TM_U32_MAX/1000) ? MSEC_TM_U32_INF : MSecTmU32(1000.*t);
}

SecTmReal golem::MSecTmU32ToSecTmReal(MSecTmU32 t) {
	return	t == MSEC_TM_U32_INF ? SEC_TM_REAL_INF : SecTmReal(t/1000.);
}

//------------------------------------------------------------------------------

#ifdef LINUX
void golem::timespecnorm(timespec& ts) {
	if (ts.tv_nsec >= 1000000000L) {
		ts.tv_sec++;
		ts.tv_nsec -= 1000000000L;
	}
	if (ts.tv_nsec < 0) {
		ts.tv_sec--;
		ts.tv_nsec += 1000000000L;
	}
}

#ifdef __CYGWIN__
	#define CLOCK_ID CLOCK_REALTIME
#else	// Cygwin
	#define CLOCK_ID CLOCK_REALTIME//CLOCK_PROCESS_CPUTIME_ID//CLOCK_HIGHRES
#endif	// Cygwin
#endif	// Linux

//------------------------------------------------------------------------------

SysTimer::SysTimer() {
#ifdef WIN32
	::timeBeginPeriod(1);
#elif LINUX
#endif
	reset();
}

SysTimer::~SysTimer() {
#ifdef WIN32
	::timeEndPeriod(1);
#elif LINUX
#endif
}

void SysTimer::reset() {
#ifdef WIN32
	stamp = ::timeGetTime();
#elif LINUX
	::clock_gettime(CLOCK_REALTIME, &stamp);
#endif
}

MSecTmU32 SysTimer::elapsed() const {
#ifdef WIN32
	return ::timeGetTime() - stamp;
#elif LINUX
	struct timespec ts;
	::clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec -= stamp.tv_sec;
	ts.tv_nsec -= stamp.tv_nsec;
	timespecnorm(ts);
	return 1000L*ts.tv_sec + ts.tv_nsec/1000000L;
#endif
}

void SysTimer::sleep(MSecTmU32 interval) {
	if (interval > 0) {
#ifdef WIN32
		::Sleep(interval);
#elif LINUX
		struct timespec t;
		t.tv_sec = (time_t)(interval/1000L);
		t.tv_nsec = (time_t)(1000000L*(interval%1000L));
		::nanosleep(&t, NULL/*no interruptions is assumed*/);
#endif
	}
}

//------------------------------------------------------------------------------

PerfTimer::PerfTimer() {
#ifdef WIN32
	::timeBeginPeriod(1);
#elif LINUX
#endif
	reset();
}

PerfTimer::~PerfTimer() {
#ifdef WIN32
	::timeEndPeriod(1);
#elif LINUX
#endif
}
	
void PerfTimer::reset() {
#ifdef WIN32
	(void)::QueryPerformanceFrequency(&sysFreq);
	(void)::QueryPerformanceCounter(&perfStamp);
#elif LINUX
	::clock_gettime(CLOCK_ID, &stamp);
#endif
}
	
SecTmReal PerfTimer::elapsed() const {
#ifdef WIN32
	LARGE_INTEGER t;
	(void)::QueryPerformanceCounter(&t);
	return (SecTmReal)(t.QuadPart - perfStamp.QuadPart)/sysFreq.QuadPart;
#elif LINUX
	struct timespec ts;
	::clock_gettime(CLOCK_ID, &ts);
	ts.tv_sec -= stamp.tv_sec;
	ts.tv_nsec -= stamp.tv_nsec;
	timespecnorm(ts);
	return (SecTmReal)ts.tv_sec + 1.0e-9*ts.tv_nsec;
#endif
}
	
void PerfTimer::sleep(SecTmReal interval) {
	if (interval > SEC_TM_REAL_ZERO) {
#ifdef WIN32
		::Sleep((U32)(interval*1.0e3));
#elif LINUX
		struct timespec t;
		t.tv_sec = (time_t)interval;
		t.tv_nsec = (time_t)(1.0e9*(interval - t.tv_sec));
		::nanosleep(&t, NULL/*no interruptions is assumed*/);
#endif
	}
}

//------------------------------------------------------------------------------

#ifdef WIN32
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000ULL
#endif

int golem::gettimeofday(struct timeval *tv, struct timezone *tz) {
	FILETIME ft;
	unsigned __int64 tmpres = 0;
	static int tzflag = 0;

	if (tv != NULL) {
		GetSystemTimeAsFileTime(&ft);

		tmpres |= ft.dwHighDateTime;
		tmpres <<= 32;
		tmpres |= ft.dwLowDateTime;

		tmpres /= 10;// convert into microseconds
		// converting file time to unix epoch
		tmpres -= DELTA_EPOCH_IN_MICROSECS; 
		tv->tv_sec = (long)(tmpres / 1000000UL);
		tv->tv_usec = (long)(tmpres % 1000000UL);
	}

	if (tz != NULL) {
		if (!tzflag) {
			_tzset();
			tzflag++;
		}
		tz->tz_minuteswest = _timezone/60;
		tz->tz_dsttime = _daylight;
	}

	return 0;
}
#endif

timeval golem::timevaldiff(const timeval& tv0, const timeval& tv1) {
	timeval tv = tv1;

	if (tv0.tv_usec < tv.tv_usec) {
		long nsec = (tv.tv_usec - tv0.tv_usec)/1000000 + 1;
		tv.tv_usec -= 1000000*nsec;
		tv.tv_sec += nsec;
	}
	if (tv0.tv_usec - tv.tv_usec > 1000000) {
		long nsec = (tv0.tv_usec - tv.tv_usec)/1000000;
		tv.tv_usec += 1000000*nsec;
		tv.tv_sec -= nsec;
	}

	tv.tv_sec = tv0.tv_sec - tv.tv_sec;
	tv.tv_usec = tv0.tv_usec - tv.tv_usec;

	return tv;
}

//------------------------------------------------------------------------------

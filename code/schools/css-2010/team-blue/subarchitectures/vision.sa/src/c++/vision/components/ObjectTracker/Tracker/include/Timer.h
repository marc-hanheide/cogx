#ifndef TIMER_H
#define TIMER_H
#pragma once

#include "headers.h"

// ***********************************************************************************


class Timer
{
private:
#ifdef WIN32
	LONGLONG m_StartTicks;		// QueryPerformance - Ticks at application start
	LONGLONG m_EndTicks;		// QueryPerformance - Ticks when calling Now()
	LONGLONG m_Frequency;		// QueryPerformance - Fequency
	double fNow;
#else	
	struct timespec AppStart, act, old;
#endif
	double m_fAppTime;			// Time since application started
	double m_fTime;				// Time between two Update calls


public:
	Timer(void);
	~Timer(void);
	
	void	Reset();
	double	Update();
	
	double	GetFrameTime(){ return m_fTime;}
	double	GetApplicationTime(){ return m_fAppTime;}
	
};

#endif


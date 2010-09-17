#ifndef TIMER_H
#define TIMER_H
#pragma once

// ***********************************************************************************
// WINDOWS FUNCTIONS
#ifdef _WIN32
#include <windows.h>

class Timer
{
private:
	LONGLONG m_StartTicks;		// QueryPerformance - Ticks at application start
	LONGLONG m_EndTicks;		// QueryPerformance - Ticks when calling Now()
	LONGLONG m_Frequency;		// QueryPerformance - Fequency
	float	fNow;
	
	double	m_fAppTime;			// Time since application started
	float	m_fTime;

public:
	Timer(void);
	~Timer(void);

	void	Reset();
	float	Update();
	
	float	GetFrameTime(){ return m_fTime;}
	float	GetApplicationTime(){ return (float)m_fAppTime;}
};
#endif


// ***********************************************************************************
// LINUX FUNCTIONS
#ifdef linux
#include <time.h>
#include <sys/time.h>
using namespace std;
class Timer
{
private:
	struct timespec AppStart, act, old;
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

#endif


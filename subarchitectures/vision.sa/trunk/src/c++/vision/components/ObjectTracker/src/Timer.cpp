
#include "Timer.h"

// ***********************************************************************************
// WINDOWS FUNCTIONS
#ifdef _WIN32
Timer::Timer(void) {
	QueryPerformanceFrequency((LARGE_INTEGER*) &m_Frequency);
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_StartTicks));
	m_EndTicks = m_StartTicks;
	m_fAppTime = 0.0f;
}

Timer::~Timer(void) {
}

void Timer::Reset() {
	QueryPerformanceFrequency((LARGE_INTEGER*) &m_Frequency);
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_StartTicks));
	m_EndTicks = m_StartTicks;
	m_fAppTime = 0.0f;
}

double Timer::Update() {
	QueryPerformanceCounter((LARGE_INTEGER*)(&m_EndTicks));
	fNow = (double)(m_EndTicks - m_StartTicks) / m_Frequency;
	m_fTime = fNow - m_fAppTime;
	m_fAppTime = fNow;
	return m_fTime;
}
#endif

// ***********************************************************************************
// LINUX FUNCTIONS
#ifdef linux
Timer::Timer(void) {
	clock_gettime(CLOCK_REALTIME, &AppStart);
	clock_gettime(CLOCK_REALTIME, &old);
	m_fAppTime = 0.0f;
}

Timer::~Timer(void) {
}

void Timer::Reset() {
	clock_gettime(CLOCK_REALTIME, &AppStart);
	clock_gettime(CLOCK_REALTIME, &old);
	m_fAppTime = 0.0f;
}

double Timer::Update() {
	clock_gettime(CLOCK_REALTIME, &act);
	m_fTime = (act.tv_sec - old.tv_sec) + (act.tv_nsec - old.tv_nsec) / 1e9;
	old = act;		
	m_fAppTime += m_fTime;
	return m_fTime;
}
#endif


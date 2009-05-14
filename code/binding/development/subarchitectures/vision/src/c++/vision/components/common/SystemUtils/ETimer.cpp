/** @file ETimer.cpp
 *  @brief Timer.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "ETimer.h"

unsigned ETimer::tick;
vector<BALTTime> ETimer::ticks;

ETimer::ETimer() 
{    
}

ETimer::~ETimer() 
{
}

void ETimer::Tick(BALTTime processingBaltTime) 
{ 
    tick++; 
    ticks.push_back(processingBaltTime);
}

BALTTime ETimer::GetProcessingBaltTime(int timetick) 
{
    return ticks[timetick];
}

/** @file ETimer.h
 *  @brief Timer.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _ETIMER_H_
#define _ETIMER_H_

#include <balt/idl/FrameworkBasics.hh>
#include <vector>

using std::vector;
using namespace FrameworkBasics;

class ETimer {
 protected:
    static unsigned tick;
    static vector<BALTTime> ticks;

 public:
    ETimer();
    virtual ~ETimer();

    static void ResetTime() { tick=0; ticks.clear();}
    unsigned GetTime() { return tick; }
    void Tick(BALTTime processingBaltTime);
    BALTTime GetProcessingBaltTime(int timetick);
};

#endif

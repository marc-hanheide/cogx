#include "CASTTimeServer.hpp"

using namespace cast::cdl;
using namespace std;

namespace cast {

    
    int CASTTimeServer::timeDiff(const timeval &_x, const timeval &_y, CASTTime &_result) {
        
        //http://www.gnu.org/software/hello/manual/libc/Elapsed-Time.html
   
        timeval y = _y;
        
        /* Perform the carry for the later subtraction by updating y. */
        if (_x.tv_usec < y.tv_usec) {
            int nsec = (y.tv_usec - _x.tv_usec) / 1000000 + 1;
            y.tv_usec -= 1000000 * nsec;
            y.tv_sec += nsec;
        }
        if (_x.tv_usec - y.tv_usec > 1000000) {
            int nsec = (y.tv_usec - _x.tv_usec) / 1000000;
            y.tv_usec += 1000000 * nsec;
            y.tv_sec -= nsec;
        }
        
        /* Compute the time remaining to wait.
	     tv_usec  is certainly positive. */
        _result.s = _x.tv_sec - y.tv_sec;
        _result.us = _x.tv_usec - y.tv_usec;
        
        /* Return 1 if result is negative. */
        return _x.tv_sec < y.tv_sec;
	}
    
       
    void CASTTimeServer::getCASTTime(const timeval & _currentTime, cdl::CASTTime & _castTime) const {
        timeDiff(_currentTime, startTime, _castTime);        
    }

    
    cast::cdl::CASTTime CASTTimeServer::getCASTTime(const ::Ice::Current& _crt) const {
        CASTTime castTime;
        timeval currentTime;
        gettimeofday(&currentTime, NULL);
        getCASTTime(currentTime,castTime);
        return castTime;
    }
    
    
    cast::cdl::CASTTime CASTTimeServer::fromTimeOfDayDouble(::Ice::Double _todsecs, const ::Ice::Current& _crt) const {
        CASTTime castTime;
        timeval currentTime;
        currentTime.tv_sec = _todsecs;
        currentTime.tv_usec = ((_todsecs - currentTime.tv_sec) * 1000000);
        getCASTTime(currentTime,castTime);
        return castTime;
        
    }
    
    cast::cdl::CASTTime CASTTimeServer::fromTimeOfDay(::Ice::Long _secs, ::Ice::Long _usecs, const ::Ice::Current& _crt) const {
        CASTTime castTime;
        timeval currentTime;
        currentTime.tv_sec = _secs;
        currentTime.tv_usec = _usecs;
        getCASTTime(currentTime,castTime);
        return castTime;
    }
    
    void CASTTimeServer::reset(const ::Ice::Current& _crt) {
        gettimeofday(&startTime, NULL);
    }

    
    //void CASTTimeServer::syncTo(::Ice::Long _secs, ::Ice::Long _usecs, const ::Ice::Current& _crt) {
//        
//    }
    

};

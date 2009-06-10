#ifndef CAST_TIME_SERVER_HPP_
#define CAST_TIME_SERVER_HPP_

#include <cast/slice/CDL.hpp>

#include <sys/time.h>


namespace cast {


  class CASTTimeServer : 
    public virtual cast::interfaces::TimeServer {
    
  public:
    virtual ~CASTTimeServer(){}
        virtual ::cast::cdl::CASTTime getCASTTime(const ::Ice::Current& = ::Ice::Current()) const;
        
        virtual ::cast::cdl::CASTTime fromTimeOfDayDouble(::Ice::Double _todsecs, const ::Ice::Current& _crt) const;

        virtual ::cast::cdl::CASTTime fromTimeOfDay(::Ice::Long _secs, ::Ice::Long _usecs, const ::Ice::Current& _crt) const;
      
   //     virtual void syncTo(::Ice::Long _secs, ::Ice::Long _usecs, const ::Ice::Current& _crt);

        virtual void reset(const ::Ice::Current& _crt);
        
        /***
         * Subtract timeval y from x, placing the result in _result;
         **/
        static int timeDiff(const timeval &_x, const timeval &_y, cdl::CASTTime &_result);
            
        /***
        * Get a CASTTime using _currentTime as the current time value.
         */
        void getCASTTime(const timeval & _currentTime, cdl::CASTTime & _castTime) const;
        
        
    private:
         timeval startTime;
  };

};


#endif



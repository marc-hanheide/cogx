/***************************************************************************
 *   Copyright (C) 2009 by Markus Bader   *
 *   markus.bader@austrian-kangaroos.com   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
/**
 * @file timehdl.cpp
 * @author Markus Bader
 * @brief
 **/

#include "timehdl.h"
#include <iostream>
#include <algorithm>
#include <cctype>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/local_time_adjustor.hpp"
#include "boost/date_time/c_local_time_adjustor.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread/thread.hpp>


namespace V4R {

FQSleep::FQSleep(const double &rFrequenz)
        : mrFrequenz(-1.0) {
    set(rFrequenz);
}
void FQSleep::set(const double &rFrequenz) {
    if (mrFrequenz != rFrequenz) {
        mrFrequenz = rFrequenz;
    }
    mrFrequenz =  rFrequenz;
    boost::xtime_get(&mXTime, boost::TIME_UTC);
    mPriod.sec = 0;
    mPriod.nsec = (1000000000.0/mrFrequenz);
    if (mPriod.nsec > 999999999) {
        mPriod.sec += mPriod.nsec / 1000000000;
        mPriod.nsec = mPriod.nsec % 1000000000;
    }
}
void FQSleep::wait() {
    boost::thread::sleep(mXTime);
    mXTime.nsec += mPriod.nsec;
    mXTime.sec += mPriod.sec;
    if (mXTime.nsec > 999999999) {
        mXTime.sec += mXTime.nsec / 1000000000;
        mXTime.nsec = mXTime.nsec % 1000000000;
    }
    /*
    boost::xtime now;
    boost::xtime_get(&now, boost::TIME_UTC);
    while (( mXTime.sec < now.sec || ( mXTime.sec == now.sec && mXTime.nsec < now.nsec ) )) {
    mXTime.nsec += mPriod.nsec;
    mXTime.sec += mPriod.sec;
    }
    */
}


boost::posix_time::ptime  &Timer::start(const std::string &rName) {
    return mStartTime[rName] = boost::posix_time::microsec_clock::local_time();
}

boost::posix_time::time_duration  &Timer::stop(const std::string &rName) {
    return mDuration[rName] = boost::posix_time::microsec_clock::local_time() - mStartTime[rName];
}

long Timer::ms(const std::string &rName) {
    return  mDuration[rName].total_milliseconds();
}


boost::posix_time::ptime  timeval2ptime_utc(unsigned long sec, unsigned long usec, unsigned long nsec) {
    using namespace boost::posix_time;
    typedef boost::date_time::subsecond_duration<time_duration,1000000000> nanoseconds;
    boost::gregorian::date d(1970, boost::gregorian::Jan, 1);
    ptime  t_utc(d, seconds(sec) + microseconds(usec) + nanoseconds(nsec));
    return t_utc;
}

boost::posix_time::ptime  timeval2ptime_utc(const timeval &t) {
    return timeval2ptime_utc(t.tv_sec, t.tv_usec, 0);
}

boost::posix_time::ptime  timeval2ptime_local(unsigned long sec, unsigned long usec, unsigned long nsec) {
    using namespace boost::posix_time;
    ptime t_utc = timeval2ptime_utc(sec, usec, nsec);
    ptime t_local = boost::date_time::c_local_adjustor<ptime>::utc_to_local(t_utc);
    return t_local;
}

boost::posix_time::ptime  timeval2ptime_local(const timeval &t) {
    return timeval2ptime_local(t.tv_sec, t.tv_usec, 0);
}

void ptime2timeval ( const boost::posix_time::ptime &src, timeval &des ) {
    boost::posix_time::ptime timet_start(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration diff = src - timet_start;
    des.tv_sec = diff.total_seconds();
    des.tv_usec = diff.fractional_seconds()/1000;
}
void ptime2timeval ( const boost::posix_time::ptime &src, uint64_t &sec_des, uint64_t &usec_des) {
    uint32_t sec, usec;
    ptime2timeval(src, sec, usec);
    sec_des = (uint64_t) sec, usec_des = (uint64_t) usec;
}

void ptime2timeval ( const boost::posix_time::ptime &src, uint32_t &sec_des, uint32_t &usec_des) {
    boost::posix_time::ptime timet_start(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration diff = src - timet_start;
    sec_des = diff.total_seconds();
    usec_des = diff.fractional_seconds()/1000;
}
timeval ptime2timeval ( const boost::posix_time::ptime &src) {
    timeval des;
    ptime2timeval(src, des);
    return des;
}
};

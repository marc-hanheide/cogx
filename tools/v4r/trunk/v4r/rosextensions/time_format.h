/**
 *  @brief header with functions to convert ros time
 *  Created on: Feb 13, 2011
 *  Author: Markus Bader
 */

#ifndef TIME_FORMAT_H_
#define TIME_FORMAT_H_

#include <ros/time.h>
#include <boost/thread/xtime.hpp>
#include "boost/date_time/c_local_time_adjustor.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>

namespace ros {


/** boost ptime to ros time
 * function to convert ros time
 * @param src ptime
 * @param des rostime
 **/
inline void time2rostime ( const boost::posix_time::ptime &src, ros::Time &des) {
    boost::posix_time::ptime timet_start(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration diff = src - timet_start;
    des.sec = diff.total_seconds();
    des.nsec = diff.total_nanoseconds() % 1000000000UL;;
}

/** boost ptime to ros time
 * function to convert ros time
 * @param src ptime
 * @return rostime
 **/
inline ros::Time time2rostime ( const boost::posix_time::ptime &src) {
    boost::posix_time::ptime timet_start(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration diff = src - timet_start;
    ros::Time des(diff.total_seconds(), diff.total_nanoseconds() % 1000000000UL);
    return des;
}

/** ros ptime to boost ptime utc
 * function to convert ros time
 * @param src rostime
 * @param des_utc ptime
 **/
inline void time2ptime_utc (const ros::Time &src,  boost::posix_time::ptime &des_utc) {
    typedef boost::date_time::subsecond_duration<boost::posix_time::time_duration,1000000000> nanoseconds;
    boost::gregorian::date d(1970, boost::gregorian::Jan, 1);
    des_utc = boost::posix_time::ptime(d, boost::posix_time::seconds(src.sec) + nanoseconds(src.nsec));
}
/** ros ptime to boost ptime utc
 * function to convert ros time
 * @param src rostime
 * @return utc time
 **/
inline boost::posix_time::ptime time2ptime_utc (const ros::Time &src) {
    boost::posix_time::ptime t_utc;
    time2ptime_utc (src,  t_utc);
    return t_utc;
}

/** ros ptime to boost ptime local
 * function to convert ros time
 * @param src rostime
 * @param des_local ptime
 **/
inline void time2ptime_local (const ros::Time &src,  boost::posix_time::ptime &des_local) {
    boost::posix_time::ptime t_utc = time2ptime_utc (src);
    des_local = boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(t_utc);
}

/** ros ptime to boost ptime local
 * function to convert ros time
 * @param src rostime
 * @return local time
 **/
inline boost::posix_time::ptime time2ptime_local (const ros::Time &src) {
    boost::posix_time::ptime t_utc = time2ptime_utc (src);
    return boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(t_utc);
}

}
#endif /* TIME_FORMAT_H_ */

/*
 * @Author: mmarko
 * @Date: 2012-05-24
 */
#include "time.hpp"
#include "dialogue.hpp"

namespace de {
namespace dfki {
namespace lt {
namespace tr {
namespace dialogue {
namespace slice {

namespace time {
   IntervalPtr newTimeInterval(long begin=0, long end=0)
   {
      if (end < begin) {
         end = begin;
      }
      IntervalPtr pti = new Interval();
      pti->begin = new TimePoint();
      pti->begin->msec = begin;
      pti->end = new TimePoint();
      pti->end->msec = end;
      return pti;
   }   
}

namespace asr { 
   PhonStringPtr newPhonString(const std::string& str = "")
   {
      PhonStringPtr pps = new PhonString();
      pps->id = "";
      pps->wordSequence = str;
      pps->confidenceValue = 1.0;
      pps->ival = time::newTimeInterval();
      return pps;
   }
}

} } } } } } // close de.dfki ... slice namespace

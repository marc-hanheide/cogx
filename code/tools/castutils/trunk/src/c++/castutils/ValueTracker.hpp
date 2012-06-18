/**
 * @author Marko Mahnič
 * @date June 2012
 *
 */
#ifndef _CASTUTILS_COUNTTRACKER_HPP_4FDF10E9_
#define _CASTUTILS_COUNTTRACKER_HPP_4FDF10E9_
#include <map>
#include <string>
#include <cmath>

namespace castutils
{

template<typename T>
class CNumericValueTracker
{
  std::map<std::string, T> mCount;
  T mTollerance;
public:
  CNumericValueTracker(T tollerance = 0)
  {
    mTollerance = std::max((T)0, tollerance);
  }

  bool sameCount(const std::string& counter, T value)
  {
    if (mCount.find(counter) == mCount.end()) {
      mCount[counter] = 0;
    }
    bool same = fabs(mCount[counter] - value) < mTollerance;
    mCount[counter] = value;
    return same;
  }
};

}
#endif /* _CASTUTILS_COUNTTRACKER_HPP_4FDF10E9_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim

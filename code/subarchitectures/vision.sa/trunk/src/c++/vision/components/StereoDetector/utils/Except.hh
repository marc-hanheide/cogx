/**
 * @file Except.hh
 * @author Michael Zillich, Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Exception class
 */

#ifndef Z_EXCEPT_HH
#define Z_EXCEPT_HH

#include <cstring>
#include <stdexcept>
#include "Namespace.hh"

/**
 * A slightly hacky way to only get the pure file name without the whole path
 * from __FILE__
 */
#define __THIS_FILE__ ((strrchr(__FILE__, '/') ?: __FILE__ - 1) + 1)
#define __HERE__   __THIS_FILE__, __FUNCTION__, __LINE__

namespace Z
{

/**
 * @brief An informative exception class.
 */
class Except : public exception
{
private:
  string _what;

public:
  Except(const char *file, const char *function, int line, const char *format, ...) throw();
  virtual ~Except() throw() {}
  virtual const char* what() const throw() {return _what.c_str();}
  void Set(const string &s) {_what = s;}
};

}

#endif


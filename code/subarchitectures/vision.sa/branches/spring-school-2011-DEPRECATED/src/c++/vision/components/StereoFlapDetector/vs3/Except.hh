/**
 * $Id: Except.hh,v 1.2 2006/11/24 13:47:03 mxz Exp mxz $
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
 * An informative exception class.
 * Example:
 *   Except(__FILE__, __FUNCTION__, __LINE__, "There were %d %s in the tree.",
 *          42, "elephants");
 * output:
 *   "DumbFile.cc:StupidFunction:28: There were 42 elephants in the tree."
 * Note: You can use the __HERE__ macro to get shorter statements:
 *   Except(__HERE__, "There were %d %s in the tree.", 42, "elephants");
 */
class Except : public exception
{
  string _what;
public:
  Except(const char *file, const char *function, int line,
         const char *format, ...) throw();
  virtual ~Except() throw() {}
  virtual const char* what() const throw() {return _what.c_str();}
  void Set(const string &s) {_what = s;}
};

}

#endif


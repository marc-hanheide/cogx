#ifndef EXCEPTION_WQJ9V10Y
#define EXCEPTION_WQJ9V10Y

#include <exception>
#include <string>
#include <sstream>

namespace cogx { namespace vision {

// Examples:
//    throw Exception("An exception with a simple string description");
//    throw Exception(EXCEPTMSG("A more complex -" << "ostringstream" << "- description"));
class Exception: public std::exception
{
   std::string _what;
public:
   Exception(const std::string& what) { _what = what; }
   Exception(std::ostringstream& what) { _what = what.str(); }
   ~Exception() throw() {}
   virtual const char* what() const throw() { return _what.c_str(); }
};
//#define EXCEPTMSG(streamexpr)  ((std::ostringstream&)(std::ostringstream() << streamexpr)).str()
#define EXCEPTMSG(streamexpr)  (std::ostringstream&)(std::ostringstream() << streamexpr)

}} // namespace
#endif /* end of include guard: EXCEPTION_WQJ9V10Y */

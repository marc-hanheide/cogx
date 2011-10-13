//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYSTRING_HPP
#define  _SYSTRING_HPP

#include <stdarg.h>
#include "multiplatform.hpp"
#include "syExcept.hpp"

NAMESPACE_CLASS_BEGIN( CzN )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       CzString
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzString
{
private:
   std::string s;
public:
   CzString() { s = string(""); }
   CzString(const char *pCharStr) { s = string(pCharStr); }
   CzString(CzString &pStr) { s = string(pStr.GetStr()); }

//   CzString& operator =( const char *pCharStr ) { s = string(pCharStr); }
   const char *GetStr() { return (char*)s.c_str(); }
   const char *GetCharStr() { return s.c_str(); }
//   operator const char*() { return s.c_str(); }
};

NAMESPACE_CLASS_END ()

#if defined ELLDETECTEXE

#else

class CString
{
private:
   CzN::CzString s;
public:
   CString() { s = ""; }
   CString(const char *pCharStr) { s = CzN::CzString(pCharStr); }

   const char *GetStr() { return s.GetStr(); }
};

class CT2CA: public std::string 
{
private: 
	 CString s;
public:
   CT2CA(CString &pStr) { s = CString(pStr); }
//   const char *GetStr() { return s.GetStr(); }
};

#endif
#endif

//
// (C) 2010, Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYSTRINGCONVERSION_HPP
#define  _SYSTRINGCONVERSION_HPP


#include <stdarg.h>
#include "multiplatform.hpp"
#include "syExcept.hpp"
#include "syString.hpp"


NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE CzStringConversion
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzStringConversion
{
private:
   CzN::CzString m_CzString;
   std::string m_String;
public:
   CzStringConversion(CzN::CzString sString);
   const char* GetStr();
   const CString NewCString();
};


NAMESPACE_CLASS_END ()

#endif
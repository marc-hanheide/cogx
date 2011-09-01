//
// (C) 2010, Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syStringConversion.hpp"
#include "syExcept.hpp"


NAMESPACE_CLASS_BEGIN( RTE )


CzStringConversion::CzStringConversion(CzN::CzString sString)
{
   m_CzString = sString;
}

const char* CzStringConversion::GetStr() 
{
   CString c_str(m_CzString.GetCharStr());
   CT2CA c_str_a(c_str);
   m_String = c_str_a;
   return m_String.c_str();
}

const CString CzStringConversion::NewCString()
{
   CString c_str(m_CzString.GetCharStr());
   return c_str;
}

NAMESPACE_CLASS_END ()

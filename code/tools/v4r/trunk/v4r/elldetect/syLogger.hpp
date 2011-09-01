//
// (C) 2010, Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYLOGGER_HPP
#define  _SYLOGGER_HPP

#include "multiplatform.hpp"
#include "syArray.hpp"
#include <string>

#include "syString.hpp"

using namespace std;

NAMESPACE_CLASS_BEGIN( RTE )

//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE file logger
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzLogger
{
private:

   ofstream *m_pOutStream;
   CzArray<struct timespec> m_IntervalCache;
   int m_iFileCreateID;
   
public:

   CzLogger();
   virtual ~CzLogger();

   // Open a file for writing
   void Init(CzN::CzString sFilename);
   // Close the file
   void Clear();

   // Write a log message
   void Log(string sMessage);
   // Remember timestamp and return ID
   void LogStart(int &iID);
   // Write a log message and extend with the time difference since LogStart was called
   void LogEnd(int iID, string sMessage);
};
//end class/////////////////////////////////////////////////////////////////////

NAMESPACE_CLASS_END()

#endif


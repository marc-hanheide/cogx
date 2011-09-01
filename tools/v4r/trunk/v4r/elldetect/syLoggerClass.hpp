//
// (C) 2010, Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYLOGGERCLASS_HPP
#define  _SYLOGGERCLASS_HPP

#include "multiplatform.hpp"

#include "syLogger.hpp"


NAMESPACE_CLASS_BEGIN( RTE )

//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE logger wrapper
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzLoggerClass
{
private:

   // pointer to global logger instance
   CzLogger *m_pLogger;
   bool m_bDoLogging;
      
public:

   CzLoggerClass();
   virtual ~CzLoggerClass();

   // Set the global logger instance
   void SetLogger(CzLogger *pLogger);

   
   // Write a log message
   void Log(string sMessage);
   // Write a formatted log message
   void Log(const char *format, ...);
   // Remember timestamp and return ID
   void LogStart(int &iID);
   // Write a log message and extend with the time difference since LogStart was called
   void LogEnd(int iID, string sMessage);
   // Write a formatted log message and extend with the time difference since LogStart was called
   void LogEnd(int iID, const char *format, ...);
   
   void SetDoLogging(bool bDoLogging);
};
//end class/////////////////////////////////////////////////////////////////////

NAMESPACE_CLASS_END()

#endif


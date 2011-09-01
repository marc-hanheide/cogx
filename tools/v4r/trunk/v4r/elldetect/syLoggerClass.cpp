//
// (C) 2010, Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syLoggerClass.hpp"


NAMESPACE_CLASS_BEGIN( RTE )


////////////////////////////////////////////////////////////////////////////////
CzLoggerClass::CzLoggerClass()
{
   m_pLogger = NULL;
   m_bDoLogging = true;
}

////////////////////////////////////////////////////////////////////////////////
CzLoggerClass::~CzLoggerClass()
{
   // the logger instance is global and has not to be freed here
   m_pLogger = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Set the global logger instance
void CzLoggerClass::SetLogger(CzLogger *pLogger)
{
   m_pLogger = pLogger;
}

////////////////////////////////////////////////////////////////////////////////
// Write a log message
void CzLoggerClass::Log(string sMessage)
{
   if (!m_bDoLogging)
      return;
   if (m_pLogger != NULL) {
      m_pLogger->Log(sMessage);
   }
}

////////////////////////////////////////////////////////////////////////////////
// Write a formatted log message
void CzLoggerClass::Log(const char *format, ...)
{
   if (!m_bDoLogging)
      return;
   char msg[1024];
   va_list arg_list;
   va_start(arg_list, format);
   vsnprintf(msg, 1024, format, arg_list);
   Log(string(msg));
   va_end(arg_list);
}

////////////////////////////////////////////////////////////////////////////////
// Remember timestamp and return ID
void CzLoggerClass::LogStart(int &iID)
{
   if (!m_bDoLogging)
      return;
   if (m_pLogger != NULL) {
      m_pLogger->LogStart(iID);
   }
}

////////////////////////////////////////////////////////////////////////////////
// Write a log message and extend with the time difference since LogStart was called
void CzLoggerClass::LogEnd(int iID, string sMessage)
{
   if (!m_bDoLogging)
      return;
   if (m_pLogger != NULL) {
      m_pLogger->LogEnd(iID, sMessage);
   }
}

////////////////////////////////////////////////////////////////////////////////
// Write a formatted log message and extend with the time difference since LogStart was called
void CzLoggerClass::LogEnd(int iID, const char *format, ...)
{
   if (!m_bDoLogging)
      return;
   char msg[1024];
   va_list arg_list;
   va_start(arg_list, format);
   vsnprintf(msg, 1024, format, arg_list);
   LogEnd(iID, string(msg));
   va_end(arg_list);
}

////////////////////////////////////////////////////////////////////////////////
void CzLoggerClass::SetDoLogging(bool bDoLogging)
{
   m_bDoLogging = bDoLogging;
}
NAMESPACE_CLASS_END()


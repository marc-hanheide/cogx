//
// (C) 2010, Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include "syLogger.hpp"

#include "SMath.h"
#include "syStringConversion.hpp"

#include <sstream>
#include <iostream>
#include <iomanip>

NAMESPACE_CLASS_BEGIN( RTE )


////////////////////////////////////////////////////////////////////////////////
CzLogger::CzLogger()
{
   m_pOutStream = NULL;
   m_iFileCreateID = -1;
}

////////////////////////////////////////////////////////////////////////////////
CzLogger::~CzLogger()
{
   Clear();
}

////////////////////////////////////////////////////////////////////////////////
// Open a file for writing
//void CzLogger::Init(string sFilename)
void CzLogger::Init(CzN::CzString sFilename)
{
   CzStringConversion s(sFilename);

   if (m_pOutStream != NULL) {
      delete m_pOutStream;
   }
   char filename[300];
   sprintf(filename, "%soutput.log", s.GetStr());
   m_pOutStream = new std::ofstream(filename, ios::out);
   if (m_pOutStream == 0)
      throw CzExcept(__HERE__, "Could not create ofstream!");
   LogStart(m_iFileCreateID);
}

////////////////////////////////////////////////////////////////////////////////
// Close the file
void CzLogger::Clear()
{
   if (m_pOutStream != NULL) {
      delete m_pOutStream;
   }
   m_IntervalCache.Clear();
   m_iFileCreateID = -1;
}

////////////////////////////////////////////////////////////////////////////////
// Write a log message
void CzLogger::Log(string sMessage)
{
   if (m_pOutStream != NULL) {
      string msg("");
      struct timespec end;
      getRealTime(&end); 
      if ((m_iFileCreateID >= 0) && (m_iFileCreateID < (int)m_IntervalCache.Size())) {
         ostringstream oss;
         oss << setfill (' ') << setw (5) << setprecision(6) << fixed << timespec_diff(&end, &m_IntervalCache[m_iFileCreateID]);
         msg.append(oss.str());
         msg.append(": ");
      }
      msg.append(sMessage);
      msg.append("\r\n");
      m_pOutStream->write(msg.c_str(), msg.length());
      m_pOutStream->flush();
#ifdef DEBUG
      TRACE(msg.c_str());
#endif
   }
}

////////////////////////////////////////////////////////////////////////////////
// Remember timestamp and return ID
void CzLogger::LogStart(int &iID)
{
   iID = m_IntervalCache.Size();
   struct timespec start;
   getRealTime(&start); 
   m_IntervalCache.PushBack(start);
}

////////////////////////////////////////////////////////////////////////////////
// Write a log message and extend with the time difference since LogStart was called
void CzLogger::LogEnd(int iID, string sMessage)
{
   struct timespec end;
   getRealTime(&end); 
   if ((iID >= 0) && (iID < (int)m_IntervalCache.Size())) {
      string msg(sMessage);
      ostringstream oss;
      oss << setprecision(6) << timespec_diff(&end, &m_IntervalCache[iID]);
      msg.append(oss.str());
      Log(msg);
   }
}

NAMESPACE_CLASS_END()


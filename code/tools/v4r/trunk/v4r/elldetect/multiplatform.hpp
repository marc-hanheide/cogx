//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _MULTIPLATFORM_HPP
#define  _MULTIPLATFORM_HPP

#if defined ELLDETECTEXE
	#include <time.h>
	//#include <windows.h>
   #include <tchar.h>
	#include "sysys.hpp"
	#include "targetver.h"

//   #define WINVER _WIN32_WINNT
   #define _AFXDLL 
	#include <afx.h>

//   #define ASSERT(value) assert(value)

//   #define TRACE(Value, ...) std::cerr<<printf(Value)

#endif

#if defined WIN32 || defined WIN64

   // compatibility definitions for windows
   
   #define OCV_CV_H "cv.h"
   #define OCV_CXCORE_H "cxcore.h"
   #define OCV_HIGHGUI_H "highgui.h"

   #define __THROW_EXCEPT 

   #define DIR_SEP "\\"

   #define snprintf(Val1, Val2, Val3, Val4, Val5, Val6, Val7) _snprintf(Val1, Val2, Val3, Val4, Val5, Val6, Val7)
   #define islessequal(Val1, Val2) ((double)(Val1) <= (double)(Val2))


   typedef struct timespec {
      time_t   tv_sec;   /* seconds */
      long     tv_nsec;  /* nanosecondes */
   } timespec;

   extern void getRealTime(struct timespec *time);

#else

   // compatibility definitions for linux

   #include "sysys.hpp"

   #define _T(Value) Value
   #define DEBUG
   #define TRACE(Value) std::cerr<<Value
   #define ASSERT(value) assert(value)
   #define DWORD int
   #define BYTE int
   #define BOOL bool
   #define UCHAR uchar
   
   #define OCV_CV_H <opencv2/imgproc/imgproc_c.h>
   #define OCV_IMGPROC_H <opencv2/imgproc/imgproc.hpp>
   #define OCV_CXCORE_H <opencv2/core/core.hpp>
   #define OCV_HIGHGUI_H <opencv2/highgui/highgui.hpp>

   #define __THROW_EXCEPT throw()

   #define DIR_SEP "/"

   #define getRealTime(Val1) clock_gettime(CLOCK_REALTIME, Val1)

#endif



#endif

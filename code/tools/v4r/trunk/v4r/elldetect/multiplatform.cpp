//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"
#include "multiplatform.hpp"

#include <stdarg.h>
#include <stdio.h>
#include <sys/timeb.h>


	#include <iostream>
	#include <stdio.h>
	#include <stdlib.h>
	//#include <sys/time.h>
	#include <time.h>
	#include <ctime>
	 
#if defined WIN32 || defined WIN64

#define ULLINT(x)  (x##ui64) /* or i64u ...*/

inline void getRealTime(struct timespec *time) {
   FILETIME t;
   GetSystemTimeAsFileTime(&t);
   // value t is precise to 100ns
/*   time_t tt = (time_t)t.dwLowDateTime + (((time_t)t.dwHighDateTime)<<32);
   time->tv_nsec = (long)(tt * (time_t)100);
   time->tv_sec  = (time_t)(tt / (time_t)10000000); */
   ULONGLONG ll = t.dwHighDateTime;   
   ll = (ll << 32) + t.dwLowDateTime;   
   time->tv_nsec = (long)(ll % 10000000) * 100;    
   time->tv_sec  = (long)(ll / 10000000 - ULLINT(11644473600));
}


#else

#endif

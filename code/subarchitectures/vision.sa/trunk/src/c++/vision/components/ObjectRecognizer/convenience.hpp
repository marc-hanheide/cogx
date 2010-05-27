/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 */

#ifndef CONVENIENCE_QEVOUBQ
#define CONVENIENCE_QEVOUBQ

#include <string>

extern std::string sfloat(double f, int precision=6);
extern double fclocks();

#ifdef DEBUG_TRACE
#include <sstream>
#include <iostream>
class __Tracer: public std::ostringstream { public:
   std::string text;
   static int level;
   int m_reportend;
   __Tracer(int reportend=1) : std::ostringstream() {
      m_reportend = reportend;
   }
   void start() {
      text = str();
      std::cout << std::string(level, ' ') << text << std::endl << std::flush;
      level++;
   }
   ~__Tracer() {
      level--;
      if (level < 0) level = 0;
      if (m_reportend)
         std::cout << std::string(level, ' ') << "END " << text << std::endl << std::flush;
   }
};
// #define DTRACE(str) std::cout << str << std::endl << std::flush
// #define DTRACE(str) __Tracer _tracerX((std::ostringstream() << str).str())
#define DTRACE(str) __Tracer _tracerX; _tracerX << str; _tracerX.start()
#define DMESSAGE(str) { __Tracer _tracerY(0); _tracerY << str; _tracerY.start(); }
#else
#define DTRACE(str) 
#define DMESSAGE(str) 
#endif

#endif /* end of include guard: CONVENIENCE_QEVOUBQ */

/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef CONVENIENCE_QEVOUBQ
#define CONVENIENCE_QEVOUBQ

#include <string>

extern std::string sfloat(double f, int precision=6);
extern double fclocks();
extern long long gethrtime(void);

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
      std::cout << std::string(level, ' ') << "|b: " << text << std::endl << std::flush;
      level++;
   }
   void message() {
      text = str();
      std::cout << std::string(level, ' ') << text << std::endl << std::flush;
      level++;
   }
   ~__Tracer() {
      level--;
      if (level < 0) level = 0;
      if (m_reportend)
         std::cout << std::string(level, ' ') << "|e: " << text << std::endl << std::flush;
   }
};
// #define DTRACE(str) std::cout << str << std::endl << std::flush
// #define DTRACE(str) __Tracer _tracerX((std::ostringstream() << str).str())
#define DTRACE(str) __Tracer _tracerX; _tracerX << str; _tracerX.start()
#define DMESSAGE(str) { __Tracer _tracerY(0); _tracerY << str; _tracerY.message(); }
#define DVERIFYGUITHREAD(str,this) { QObject test; if (test.thread() != this->thread()) {\
   DMESSAGE("********* " << str << " will be created in another thread.") }}
#else
#define DTRACE(str) 
#define DMESSAGE(str) 
#define DVERIFYGUITHREAD(str,this)
#endif

#endif /* end of include guard: CONVENIENCE_QEVOUBQ */

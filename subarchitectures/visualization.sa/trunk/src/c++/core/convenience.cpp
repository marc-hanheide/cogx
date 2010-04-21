/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 */

#include "convenience.hpp"

#include <sstream>
#include <iomanip>
#include <ctime>

#ifdef DEBUG_TRACE
int __Tracer::level = 0;
#endif

std::string sfloat(double f, int precision)
{
   std::ostringstream out;
   out << std::fixed << std::setprecision(precision) << f;
   return out.str();
}

double fclocks()
{
   return 1.0 * clock() / CLOCKS_PER_SEC;
}

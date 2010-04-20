/*
 * Author: Marko Mahnič
 * Created: 2010-04-16
 */

#ifndef CONVENIENCE_QEVOUBQ
#define CONVENIENCE_QEVOUBQ

#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>

std::string sfloat(double f, int precision=6)
{
   std::ostringstream out;
   out << std::fixed << std::setprecision(precision) << f;
   return out.str();
}

double fclocks()
{
   return 1.0 * clock() / CLOCKS_PER_SEC;
}

#endif /* end of include guard: CONVENIENCE_QEVOUBQ */

#ifndef CONSOLEUTILS_H__
#define CONSOLEUTILS_H__  1

#include <iostream>

namespace tty {

std::ostream & dcol(std::ostream & out);

std::ostream & green(std::ostream & out);
std::ostream & yellow(std::ostream & out);
std::ostream & white(std::ostream & out);

};

#endif

#ifndef _IO_UTIL_H_
#define _IO_UTIL_H_

#include <fstream>

using namespace std;

void writeString(ofstream &ofile, string ostr);
void readString(ifstream &ifile, string &istr);


#endif

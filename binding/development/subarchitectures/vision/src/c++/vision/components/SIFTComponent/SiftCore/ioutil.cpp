#include "ioutil.h"

void writeString(ofstream &ofile, string ostr) {
  int strlength = ostr.length();  
  ofile.write((char*)&strlength, sizeof(strlength));
  char *cstr = new char[strlength];
  strcpy(cstr, ostr.c_str());
  ofile.write(cstr, strlength);
  delete(cstr);
}


void readString(ifstream &ifile, string &istr) {
  int strlength;
  ifile.read((char*)&strlength, sizeof(strlength));
  char *mystr = new char[strlength+1];
  ifile.read((char*)mystr, strlength);
  mystr[strlength] = '\0';  
  istr = mystr;
  delete(mystr);
}

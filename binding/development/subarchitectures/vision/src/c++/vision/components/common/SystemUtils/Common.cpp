/** @file Common.cpp
 *  @brief Common utility and error handling functions.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include <cstdarg>
#include <iostream>
#include <libgen.h>
#include "Common.h"

using namespace std;

void Common::user_assert(bool expression, 
			 const char *file, const char *function, int line,
			 const char* message) // may throw exception
{
  if (!expression) 
  {
      std::cout << "Assertion failed!";
      throw user_error(file, function, line, message);
//       abort();
  }
}

void Common::user_printf(const char *file, const char *function, int line,
			 const char *format, ...) 
{
    char what[1024];
    char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(what, 1024, format, arg_list);
    va_end(arg_list);
    
    string pathname = file;
    int pos = pathname.find_last_of("/");
    string classname = pathname.substr(pos+1);

    snprintf(msg, 1024, "[%s:%s:%d]: %s", classname.c_str(), function, line, what);
    std::cout << msg;
}


Common::user_error::user_error(const char *file, 
			       const char *function, int line,
			       const char *format, ...) throw() // don't throw anything
{
    static char what[1024];
    static char msg[1024];
    va_list arg_list;
    va_start(arg_list, format);
    vsnprintf(what, 1024, format, arg_list);
    va_end(arg_list);
    snprintf(msg, 1024, "%s:%s:%d: %s", file, function, line, what);
    _what = msg;
    printf(msg);  
}

void Common::user_error::addInfo(const std::string& info) {
    printf(info.c_str());
}


/**
 *  Construct a complete filename by prefix the filename with
 *  the current directory from "cpath_filename".
 */
string Common::completeFilename(string cpath_filename, string filename) 
{
    char cpath_file[1024];
    snprintf(cpath_file, 1024, "%s", cpath_filename.c_str());
    string path = dirname(cpath_file);
    string complete_filename = path + "/" + filename.c_str();
    return complete_filename;
}

void Common::ParseConfigFile(string cfgFilename, map<string,string> &config)
{
     ifstream ifile(cfgFilename.c_str());
     string strKey;
     string strValue;

     if (!ifile.good())
	 throw user_error(__HERE__, "config file %s error", 
			  cfgFilename.c_str());
     else {  //must check the vilidity of content format, try-catch exception
	 while(!ifile.eof()) {
	     ifile >> strKey >> strValue;
	     config[strKey] = strValue;
	 }
	 ifile.close();
     }
 }


bool Common::GetNextLine(ifstream &ifile, string &line)
{
    while (ifile.eof() == false) {
	string strLine;
	getline(ifile, strLine);
	if (strLine[0] != '#') {
	    line = strLine;
	    return true;
	}
    }
    return false;
}

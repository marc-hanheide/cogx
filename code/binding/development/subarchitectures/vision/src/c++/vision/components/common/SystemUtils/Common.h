/** @file Common.h
 *  @brief Common utility and error handling functions.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _COMMON_H
#define _COMMON_H

#include <string>
#include <exception>
#include <map>
#include <fstream>

namespace Common {
    
#ifndef __HERE__ 
#define __HERE__   __FILE__, __FUNCTION__, __LINE__
#endif

    using namespace std;

    class user_error : public std::exception  
	{
	private:
	    string _what;
	public:
	    user_error(const char *file, const char *function, int line,
		       const char *format, ...) throw();
	    virtual ~user_error() throw() {}
	    virtual const char* what() const throw() {return _what.c_str();}

	    void addInfo(const string& info);
	};

    void user_assert(bool expression, 
		     const char *file, const char *function, int line,
		     const char* message=0);
    
    void user_printf(const char *file, const char *function, int line,
		     const char *format, ...);

    // Construct a complete filename 
    string completeFilename(string cpath_filename, string filename);

    void ParseConfigFile(string cfgFilename, 
			 map<string,string> &config);
    
    // Get the next line, but skip the comment line (starting wth #)
    bool GetNextLine(ifstream &ifile, string &line);
}

#endif

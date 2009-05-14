
#include <string>
#include <iostream>
#include <libCmd/StringOption.h>

using namespace Cmd_ns;
using namespace std;


StringOption::StringOption(string name, string helptext, bool req /*=false*/):
		CmdOption(name, helptext, req){
}

StringOption::StringOption(string name, string helptext, const char* defval):
		CmdOption(name, helptext, defval){
}

int StringOption::setvalue(const char ** options, int size){
	if(size>0){
		value=options[0];
		return 1;
	}
	return -1; //error
}

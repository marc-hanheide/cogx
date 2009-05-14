
#include <string>
#include <iostream>
#include <libCmd/BoolOption.h>

using namespace Cmd_ns;
using namespace std;


BoolOption::BoolOption(string name, string helptext):
		CmdOption(name, helptext, false),value(false){
}

int BoolOption::setvalue(const char **, int){
	value=true;
	return 0;
}

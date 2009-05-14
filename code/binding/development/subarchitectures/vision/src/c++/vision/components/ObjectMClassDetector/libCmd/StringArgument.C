
#include <string>
#include <iostream>
#include <libCmd/StringArgument.h>

using namespace Cmd_ns;
using namespace std;


StringArgument::StringArgument(string name, string helptext, bool /*req =false*/):
		CmdOption(name, helptext){
			/*if(req){
				cerr << "Optinal arguments not supported yet" << endl;
				exit(1);
			}*/
}

StringArgument::StringArgument(string name, string helptext, const char* /*defaultval*/):
		CmdOption(name, helptext){
			//if(!required){
				cerr << "Optinal arguments not supported yet" << endl;
				exit(1);
			//}
}

int StringArgument::setvalue(const char ** options, int size){
	if(size>0){
		value=options[0];
		return 1;
	}
	return -1; //error
}

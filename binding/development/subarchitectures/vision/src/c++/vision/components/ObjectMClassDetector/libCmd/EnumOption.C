
#include <iostream>
#include <algorithm>
#include <functional>
#include <iterator>
#include <libCmd/EnumOption.h>

using namespace Cmd_ns;
using namespace std;


EnumOption::EnumOption(std::string name, std::string helptext, std::string range, char** ehelp /*=NULL*/):
	CmdOption(name, helptext, true){
		range2choices(range);
		if(ehelp) initehelps(ehelp);
	}

EnumOption::EnumOption(std::string name, std::string helptext, std::string range, const char* defval, char** ehelp):
	CmdOption(name, helptext, defval){
		range2choices(range);
		if(ehelp) initehelps(ehelp);
	}

void EnumOption::initehelps(char** h){
	copy(&h[0],&h[choices.size()],back_inserter(ehelps));
}

void EnumOption::range2choices(std::string range){
	unsigned int i=0;
	//cout << "Range2choices with : " << range << endl;

	while(i<range.size()){
		int j=0;
		while ( (range[i+j]==' ') && ((i+j)<range.size()) ) ++i;
		if( (i+j)<range.size() ){
			while ( ((i+j)<range.size()) && (range[i+j]!=' ') ) ++j;
			choices.push_back(range.substr(i,j));
		}
		i+=j;
	}
	//cout << "";
}

int EnumOption::setvalue(const char ** options, int size){
	if(size>0){
		bool found=false;
		for(unsigned int i=0; i<choices.size(); ++i)
			if(ismatch(choices[i].c_str(),options[0])){
				if(found){
					cerr << "Error: the value '" << options[0] << "' is not unique for the option." << endl;
					return -1; //error
				}
				found=true;
				value=i;
			}
		return (found ? 1 : -1);
	}
	return -1; //error
}

void EnumOption::printshortformparam(std::ostream& out) const { 
	out << " ";
	vector<string>::const_iterator i=choices.begin();
	out << (*i++); // -- Case of Segfault: There is no enum values saved.
	while(i!=choices.end()) out << "|" << (*i++);
}

void EnumOption::printhelp(std::ostream& out) const{
	out << endl << "    Possible values: ";
	if(ehelps.size()>0){
			vector<string>::const_iterator i=choices.begin();
			vector<string>::const_iterator h=ehelps.begin();
			while(i!=choices.end()) 
				out << endl << "      " << (*i++) << ": " << (*h++);
	} else {
		vector<string>::const_iterator i=choices.begin();
		out << (*i++); // -- Case of Segfault: There is no enum values saved.
		while(i!=choices.end()) out << "," << (*i++);
	}
}

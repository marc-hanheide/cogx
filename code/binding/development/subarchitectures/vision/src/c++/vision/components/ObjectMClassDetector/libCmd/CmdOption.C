#include <string>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <functional>
#include <libCmd/CmdOption.h>
#include <libCmd/CmdParser.h>

using namespace Cmd_ns;
using namespace std;

// option constructors
CmdOption::CmdOption(string n, string h, bool req):
	name(n), helptext(h), required(req),specified(false), arg(false), hasdef(false), hiddenhelp(false), hiddenvalue(false){
		CmdParser::addoption(this);
}
CmdOption::CmdOption(string n, string h, const char* defval):
	name(n), helptext(h), defaultval(defval), required(false),specified(false), arg(false), hasdef(true), hiddenhelp(false), hiddenvalue(false){
		CmdParser::addoption(this);
}

// argument constructor
CmdOption::CmdOption(string n, string h):
	name(n), helptext(h), required(true),specified(false), arg(true), hasdef(false){
		CmdParser::addoption(this);
}

bool CmdOption::isit(string s) const {
	return (ismatch(name.c_str(),s.c_str()));
}

bool CmdOption::ismatch(const char* base, const char* value){
	// to make it ase insensitive
	string u(base);
	transform(u.begin(), u.end(), u.begin(), ::tolower);
	string s(value);
	std::transform(s.begin(), s.end(), s.begin(), ::tolower);

	return ( strncmp(u.c_str(),s.c_str(),s.size()) == 0 );
}

void CmdOption::printhelphead(ostream& out) const{
	if(hiddenhelp) return;
	out << "  " << (arg ? " " : "-") << name << ' ';
	if(arg){
		out << "(argument";
		if(!required)
			out << ", optional";
	} else {
		out << "(option";
		if(required) 
			out << ", required";
	}
	out << "): " << helptext;
	printhelp(out);
	if(hasdef){
		out << endl << "    default: " << defaultval;
	}
	out << endl;
}

void CmdOption::printvaluehead(ostream& out) const{
	if(hiddenvalue) return;
	printhelphead(out);
	out << "    value";
	//if(hasdefault())
		//out << " (has default)";
	out << ": ";
	printvalue(out);
	out << endl;
}

void CmdOption::printfullshortform(ostream& out) const{
	if(hiddenhelp) return;
	if(!required) out << '['; else out << ' ';
	if(arg) 
		out << '<' << name << '>';
	else {
		out << '-' << name;
		printshortformparam(out);
	}
	if(!required) out << ']'; else out << ' ';
}

void CmdOption::hidevalue(){
	hiddenvalue=true;
}

void CmdOption::hidehelp(){
	hiddenhelp=true;
}

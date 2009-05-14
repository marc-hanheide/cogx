#ifndef ENUM_OPTION_H
#define ENUM_OPTION_H

#include <string>
#include <vector>
#include <libCmd/CmdOption.h>

namespace Cmd_ns{

class EnumOption : public CmdOption {

	public:

		typedef int value_type;

		EnumOption(std::string name, std::string helptext, std::string range, char** ehelp=NULL); // always required
		EnumOption(std::string name, std::string helptext, std::string range, const char* defval, char** ehelp=NULL); // not-required

		virtual int setvalue(const char ** options, int size);
		virtual value_type getvalue() const { return value; }
		virtual void printvalue(std::ostream& out) const { out << choices[value]; }
		virtual void printshortformparam(std::ostream& out) const;
		virtual void printhelp(std::ostream& out) const;

		operator value_type () const { return value; }
		operator const char* () const { return choices[value].c_str(); }

	private:
		void range2choices(std::string range);
		void initehelps(char** h);

		value_type value;
		std::vector<std::string> choices;
		std::vector<std::string> ehelps;

}; // class EnumOption

}; // Cmd_ns
#endif

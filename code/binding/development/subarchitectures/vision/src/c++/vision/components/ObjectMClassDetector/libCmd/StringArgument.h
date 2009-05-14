#ifndef STRING_ARGUMENT_H
#define STRING_ARGUMENT_H

#include <libCmd/CmdOption.h>

namespace Cmd_ns{
class StringArgument : public CmdOption{

	public:

	typedef std::string value_type;

	StringArgument(std::string name, std::string helptext, bool req=false);
	StringArgument(std::string name, std::string helptext, const char* defval);

	virtual int setvalue(const char ** options, int size);
	virtual value_type getvalue() const { return value; }
	virtual void printvalue(std::ostream& out) const { out << value; };
	virtual void printshortformparam(std::ostream& out) const { out << " <string>";};

	operator value_type () const { return value; }
	operator const char* () const { return value.c_str(); }

	private:
	
		value_type value;


}; // StringArgument
}; // Cmd_ns

#endif

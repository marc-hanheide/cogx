#ifndef BOOL_OPTION_H
#define BOOL_OPTION_H

#include <libCmd/CmdOption.h>
#include <iostream>

namespace Cmd_ns{
class BoolOption : public CmdOption{

	public:

	typedef bool value_type;

	BoolOption(std::string name, std::string helptext);

	virtual int setvalue(const char ** options, int size);
	virtual void setvalue(bool v) { value=v; }
	virtual value_type getvalue() const { return value; }
	virtual void printvalue(std::ostream& out) const { out << ( value ? "true" : "false");};


	operator value_type () const { return value; }

	private:
	
		value_type value;


}; // BoolOption
} // Cmd_ns

#endif

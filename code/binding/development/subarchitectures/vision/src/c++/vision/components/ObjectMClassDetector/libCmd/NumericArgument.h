#ifndef NUMERIC_ARGUMENT_H
#define NUMERIC_ARGUMENT_H

#include <iostream>
#include <sstream>
#include <string>
#include <libCmd/CmdOption.h>

namespace Cmd_ns{

template<typename T>
class NumericArgument : public CmdOption{

	public:

	typedef T value_type;

	NumericArgument(std::string name, std::string helptext, T min, T max, bool req=false);
	NumericArgument(std::string name, std::string helptext, T min, T max, const char* defval);

	virtual int setvalue(const char ** options, int size);
	virtual void setvalue(value_type v) { value=v; };
	virtual value_type getvalue() const { return value; };
	virtual void printvalue(std::ostream& out) const { out << value; };
	virtual void printshortformparam(std::ostream& out) const { out << " <number>";};
	virtual void printhelp(std::ostream& out) const;

	operator value_type () const { return value; }

	protected:
	
		value_type value;
		value_type minval;
		value_type maxval;


}; // NumericArgument

typedef NumericArgument<int> IntArgument;
typedef NumericArgument<unsigned int> UIntArgument;
typedef NumericArgument<long> LongArgument;
typedef NumericArgument<double> DoubleArgument;
typedef NumericArgument<float> FloatArgument;

} // Cmd_ns

#include "NumericArgument.hh"

#endif

#ifndef NUMERIC_OPTION_H
#define NUMERIC_OPTION_H

#include <iostream>
#include <sstream>
#include <string>
#include <libCmd/CmdOption.h>

namespace Cmd_ns{

template<typename T>
class NumericOption : public CmdOption{

	public:

	typedef T value_type;

	NumericOption(std::string name, std::string helptext, T min, T max, bool req=false);
	NumericOption(std::string name, std::string helptext, T min, T max, const char* defval);

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


}; // NumericOption

typedef NumericOption<int> IntOption;
typedef NumericOption<unsigned int> UIntOption;
typedef NumericOption<long> LongOption;
typedef NumericOption<double> DoubleOption;
typedef NumericOption<float> FloatOption;

} // Cmd_ns

#include "NumericOption.hh"

#endif

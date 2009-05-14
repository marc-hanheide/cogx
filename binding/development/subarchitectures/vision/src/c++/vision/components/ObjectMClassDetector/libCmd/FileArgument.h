#ifndef FILE_ARGUMENT_H
#define FILE_ARGUMENT_H

#include <libCmd/CmdOption.h>
#include <libCmd/StringArgument.h>

namespace Cmd_ns{
class FileArgument : public StringArgument{

	public:

	typedef std::string value_type;

	FileArgument(std::string name, std::string helptext, bool req=false): StringArgument(name, helptext, req) {}
	FileArgument(std::string name, std::string helptext, const char* defval): StringArgument(name, helptext, defval) {}

	virtual void printshortformparam(std::ostream& out) const { out << " <file name>";};

}; // FileArgument
}; // Cmd_ns

#endif

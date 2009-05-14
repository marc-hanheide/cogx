#ifndef FILE_OPTION_H
#define FILE_OPTION_H

#include <libCmd/StringOption.h>

namespace Cmd_ns{
class FileOption : public StringOption{
	public:
		FileOption(std::string name, std::string helptext, bool req=false): StringOption(name, helptext, req){}
		FileOption(std::string name, std::string helptext, const char* defval): StringOption(name, helptext, defval) {}
		virtual void printshortformparam(std::ostream& out) const { out << " <file name>";};
}; // FileOption
}; // Cmd_ns

#endif

#ifndef CMDPARSER_H
#define CMDPARSER_H

#include <iostream>
#include <list>
#include <string>

namespace Cmd_ns{
class CmdParser;
}

#include <libCmd/CmdOption.h>
//#include <libCmd/License.h>

namespace Cmd_ns{

	std::ostream& operator<< (std::ostream &out, const CmdParser& parser);

	class CmdParser {
		public:
			typedef std::list<CmdOption*> OptionList;


		private:
			int processarg(int argc, char** argv, int pos);
			int processopt(int argc, char** argv, int pos);
			CmdParser(char** hlp, int argc, char** argv); // automatically process the arguments/options
			std::string prgname; //!< the name of the program
			static OptionList& getoptions();

		public:
			friend std::ostream& operator<< (std::ostream &out, const CmdParser& parser);

			static void addoption(Cmd_ns::CmdOption* opt);
			//static void removeoption() {getoptions().pop_back();}
			static CmdParser ProcessArgs(char** hlp, int argc, char** argv);
	};

	//std::ostream& operator<< (std::ostream &out, const CmdParser& img);

}; // Cmd_ns

#endif // PARSER_H


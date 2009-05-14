#ifndef CMDOPTION_H
#define CMDOPTION_H

#include <string>

namespace Cmd_ns{
class CmdOption;
}

#include <libCmd/CmdParser.h>


namespace Cmd_ns{

	class CmdOption {
		public:

			friend class CmdParser;

			//! option constructor
			CmdOption(std::string name, std::string helptext, bool req);
			CmdOption(std::string name, std::string helptext, const char* defval);

			//! Argument constructor
			CmdOption(std::string name, std::string helptext);

			virtual ~CmdOption(){}

			//! give a value to the option, return Error std::string, or null if ok
			//! Return the new position (the next unprocessed)
			virtual int setvalue(const char ** options, int size)=0;

			bool isspecified() const { return specified; }
			bool isarg() const { return arg; }
			bool isit(std::string s) const;
			bool isrequired() const { return required; }
			bool hasdefault()  const{ return hasdef; }

			static bool ismatch(const char* base, const char* value);

			std::string getname() const { return name; }
			std::string gethelp() const { return helptext; }

			virtual void printhelphead(std::ostream& out) const;
			virtual void printhelp(std::ostream&) const {};

			virtual void printvaluehead(std::ostream& out) const;
			virtual void printvalue(std::ostream&) const {};

			virtual void printfullshortform(std::ostream& out) const;
			virtual void printshortformparam(std::ostream&) const {};

			void hidevalue();
			void hidehelp();

			operator bool () const { return specified; }

		private:
			std::string name;
			std::string helptext;
			std::string defaultval;
			bool required;
			bool specified;
		protected:
			bool arg; //!< true if it is an argument (not an option)
		private:
			bool hasdef;
			bool hiddenhelp;
			bool hiddenvalue;
	};

} // Cmd_ns

#endif // CMDOPTION_H


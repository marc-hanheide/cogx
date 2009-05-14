#include <iostream>
#include <libCmd/CmdParser.h>
#include <libCmd/BoolOption.h>


using namespace Cmd_ns;
using namespace std;

CmdParser CmdParser::ProcessArgs(char ** hlp, int argc, char** argv){
	static bool singleton=false;
	if(singleton){
		cerr << "Error: CmdParser cannot created twice in one porgram." << endl;
		exit(1);
	}
	singleton=true;
	return CmdParser(hlp,argc,argv);
}

CmdParser::CmdParser(char** hlp, int argc, char** argv)
	:prgname(argv[0]){
		// TODO: hlp
		//
	
	static BoolOption helpo("help", "display help");
	static BoolOption info("printoptions","output the current setting of parameters first");
	static BoolOption licenseo("license", "display license information");


	licenseo.hidehelp();
	licenseo.hidevalue();
	helpo.hidehelp();
	helpo.hidevalue();
	info.hidehelp();
	info.hidevalue();

	int n;
	int i=1;
	while(i<argc){
		if( (argv[i][0] != '-') || (argv[i][1] == '\0') ){
			n=processarg(argc, argv, i);
			if(n<1){
				cerr << "Error processing argument value:" << argv[i] << endl;
				if(helpo || licenseo) n=argc; else exit(1);
			}
		} else {
			n=processopt(argc, argv,i);
			if(n<1){
				cerr << "Error processing option:" << argv[i] << endl;
				if(helpo || licenseo) n=argc; else exit(1);
			}
		}
		i+=n;

	}

	if(licenseo){
		// licensetext
		for (i=0; hlp[i]!=NULL; ++i) ; ++i;
		cout << endl << "--- LICENSE BEGIN ---" << endl;
		for (; hlp[i]!=NULL; ++i) cout << hlp[i] << endl;
		cout << endl << "--- LICENSE END ---" << endl;
		exit(0);
	}

	if(helpo){
		// syntax
		cout << "Syntax:" << endl << "  " << prgname << " ";
		OptionList::iterator o=getoptions().begin();
		while(o!=getoptions().end()){
			if (!(*o)->isarg()) (*o)->printfullshortform(cout);
			++o;
		}
		o=getoptions().begin();
		while(o!=getoptions().end()){
			if ((*o)->isarg()) (*o)->printfullshortform(cout);
			++o;
		}
		// helptext
		cout << endl << "Description: " << endl;
		for (i=0; hlp[i]!=NULL; ++i) cout << "  " << hlp[i] << endl;
		// parameters
		cout << "Command-line options:" << endl; 
		o=getoptions().begin();
		while(o!=getoptions().end()){
			if (!(*o)->isarg()) (*o)->printhelphead(cout);
			++o;
		}
		// Arguments
		cout << "Arguments:" << endl; 
		o=getoptions().begin();
		while(o!=getoptions().end()){
			if ((*o)->isarg()) (*o)->printhelphead(cout);
			++o;
		}
		exit(0);
	}

	// check for required options
	OptionList::iterator o=getoptions().begin();
	while(o!=getoptions().end()){
		if(!(*o)->isspecified()){
			if((*o)->isrequired()){
					if((*o)->isarg()){
						cerr << "Error: Missing Argument '" << (*o)->getname() << "'" << endl;
					} else {
						cerr << "Error: Missing required option: " << (*o)->getname() << endl;
					}
					cerr << "For help run: " << endl << '\t' << prgname << " -help" << endl;
					exit(1);
			} else if((*o)->hasdefault()){
					char* dv=new char[255];
					strcpy(dv,(*o)->defaultval.c_str());
					if ((*o)->setvalue(const_cast<const char**>(&dv), 1) < 1){
						cerr << "Error using default value for: " << (*o)->name << endl;
						exit(1);
					}
					delete[] dv;
			}
		}
		++o;
	}
	if(info)
		cout << (*this);
		//OptionList::iterator o=options.begin();
		//while(o!=options.end())
		//	(*o++)->printvaluehead(cout);
}

int CmdParser::processarg(int argc, char** argv, int pos){
	OptionList::iterator o=getoptions().begin();
	while(o!=getoptions().end()){
		if((*o)->isarg() && !(*o)->isspecified()){
			(*o)->specified=true;
			return (*o)->setvalue(const_cast<const char**>(argv+pos), argc-pos);
		}
		++o;
	}
	return -1;
}

int CmdParser::processopt(int argc, char** argv, int pos){
	OptionList::iterator o=getoptions().begin();
	bool found=false;
	int n=-1;
	while(o!=getoptions().end()){
		if(!(*o)->isspecified()){
			if((*o)->isit(&(argv[pos][1]))){
				if(found){
					cerr << "Error: Option name '" << argv[pos] << "' is not unique." << endl;
					exit(1);
				}
				found=true;
				n=(*o)->setvalue(const_cast<const char**>(argv+pos+1), argc-pos-1);
				if(n>=0) (*o)->specified=true;
			}
		}
		++o;
	}
	return n+1;
}

CmdParser::OptionList& CmdParser::getoptions() {
	static OptionList options; //!< the list of options
	return options; 
}

void CmdParser::addoption(Cmd_ns::CmdOption* opt) {
	static std::string lastname;
	if(lastname==opt->getname()) getoptions().pop_back();
	lastname=opt->getname();
	getoptions().push_back(opt);
}

std::ostream& Cmd_ns::operator<< (std::ostream &out, const CmdParser& parser){
	out << "Program name: " << parser.prgname << endl;
	CmdParser::OptionList::iterator o=parser.getoptions().begin();
	while(o!=parser.getoptions().end())
		(*o++)->printvaluehead(out);
	return out;
}


//CmdParser::OptionList CmdParser::options;

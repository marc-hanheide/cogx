// imaplementation of NumericOption<T>
// Should never be included directly.
// NumericOption.h inlcudes this file

template<typename T>
Cmd_ns::NumericOption<T>::NumericOption(std::string name, std::string helptext, T min, T max, bool req /*=false*/):
	CmdOption(name, helptext, req), minval(min), maxval(max){
}

template<typename T>
Cmd_ns::NumericOption<T>::NumericOption(std::string name, std::string helptext, T min, T max, const char* defval):
		CmdOption(name, helptext, defval), minval(min), maxval(max){
}

template<typename T>
int Cmd_ns::NumericOption<T>::setvalue(const char ** options, int size){
	if(size>0){
		std::istringstream i(options[0]);
		if (i >> value){
			// Range check
			if ( (value<minval) || (value>maxval) ){
				std::cerr << "Error: Not in Range" << std::endl;
				return -1;
			}
			return 1;
		}
		std::cerr << "Error: Not a number." << std::endl;
	}
	return -1; //error
}

template<typename T>
void Cmd_ns::NumericOption<T>::printhelp(std::ostream& out) const{
	out << std::endl <<
		"    Minimum value: " << minval <<std::endl <<
		"    Maximum value: " << maxval;
}


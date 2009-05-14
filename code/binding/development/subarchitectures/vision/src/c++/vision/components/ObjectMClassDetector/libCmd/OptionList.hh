#include <limits>
#include <iostream>



// this is a trick -- we can have it temporarly
template<typename VT> struct option_limits:std::numeric_limits<VT>{ };
template<> struct option_limits<std::string>{
      static const bool is_specialized = true;
      static std::string min() throw() { return "MIN"; }
      static std::string max() throw() { return "MAX"; }
};

template<typename U, typename V> struct option_limits<std::vector<U,V> >{
      static const bool is_specialized = true;
      static std::vector<U,V> min() throw() { return std::vector<U,V>(); }
      static std::vector<U,V> max() throw() { return std::vector<U,V>(); }
};

template<typename T> static void clearoption(T&) {}

template<typename T>
static void clearoption(Cmd_ns::OptionList<T>& t){
	t.clear();
}

/*
template<class T> 
struct option_limits<Cmd_ns::OptionList<T> >{
      static const bool is_specialized = true;
      static std::vector<typename T::value_type> min() throw() { return std::vector<typename T::value_type>(); }
      static std::vector<typename T::value_type> max() throw() { return std::vector<typename T::value_type>(); }
};
*/

template<typename T>
Cmd_ns::OptionList<T>::OptionList(T basecmdoption, bool sm/*=true*/): 
	StringOption(basecmdoption.getname(), basecmdoption.gethelp()+"(with @, can be a listfile)", basecmdoption.isrequired()), option(basecmdoption), 
		min(option_limits<typename T::value_type>::max()),
		max(option_limits<typename T::value_type>::min()), spacemulti(sm), magicspace(OptionList_trait<T>::haschild && !sm){
		if(option.isarg()){
			arg=true;
		}
	basecmdoption.hidehelp();
	basecmdoption.hidevalue();
}

template<typename T>
int Cmd_ns::OptionList<T>::setvalue(const char* so){
	//const char* q=so;
	return setvalue(&so,1);
}

template<typename T>
int Cmd_ns::OptionList<T>::setvalue(const char ** options, int size){
	using namespace std;
	if(size>0){
		value=options[0];
		if(magicspace && (value.find(' ')<value.size())) {
			spacemulti=true;
			magicspace=false;
		}
		if(options[0][0]=='@' && (!spacemulti) ){
			// each line of a file is one sub-option
			string filename=&options[0][1];

			// look for ':' modifier in the filename
			int startpos=0; // from the first line
			int limit=-1; // 1 - no limit
			size_t mpos=filename.find(':');
			if(mpos!=string::npos){
				if (istringstream(filename.substr(mpos+1)) >> startpos){
					size_t m2pos=filename.find(':',mpos+1);
					if(m2pos!=string::npos){
						if (!(istringstream(filename.substr(m2pos+1)) >> limit)){
							cerr << "Error: Invalid listfile limit modifier in " << filename << endl;
							return -1; // error
						}
					}
				} else {
					cerr << "Error: Invalid listfile statpos modifier in " << filename << endl;
					return -1; // error
				}
				filename.resize(mpos);
			}
			// check for prefix strings
			string prefix;
			string prefix_casc;
			mpos=filename.rfind('^');
			while(mpos!=string::npos){
				prefix+=filename.substr(mpos+1);
				if(filename[mpos-1]=='^'){
					prefix_casc+=filename.substr(mpos+1);
					filename.resize(mpos-1);
				} else {
					filename.resize(mpos);
				}
				mpos=filename.rfind('^');
			}
			if(!prefix_casc.empty()) prefix_casc="^^"+prefix_casc;
			//cout << "Prefix: " << prefix << endl;
			//cout << "Prefix cascaded: " << prefix_casc << endl;
			ifstream in(filename.c_str());
			if(in){
				string item;
				getline(in,item);
				while(in && (limit!=0) ){
					if(startpos>0){
						--startpos;
						getline(in,item);
					} else {
						const char* s=item.c_str();
						if((s[0]=='@') && !OptionList_trait<T>::external_AT){
							//cout << "sublist found: " << s << endl;
							string item2(item);
							item2.insert(item2.find(':'),prefix_casc);
							s=item2.c_str();
							if(setvalue(&s,1)!=1){
								cerr << "Error in listfile: " << filename << endl;
								cerr << "  while interpreting embeded list in line: " << item << endl;
								return -1; //error
							}
							getline(in,item);
							--limit;
						} else {
							string item2;
							if(s[0]=='@'){
								item2=item;
								item2.insert(item2.find(':'),prefix_casc);
							} else {
								item2=prefix+item;
							}
							s=item2.c_str();
							//cout << "Modified item: " << s << endl;
							if(option.setvalue(&s,1)!=1){
								if(item==""){
									cerr << "Warning: Empty line in listfile " << filename << endl;
									if(limit>0) cerr << "         Not counted in the limit!" << filename << endl;
									getline(in,item);
								} else {
									cerr << "Error in listfile: " << filename << endl;
									cerr << "  in line: " << item << endl;
									return -1; //error
								}
							} else {
								newvalue(option);
								clearoption(option);
								getline(in,item);
								--limit;
							}
						}
					}
				}
			} else {
				cerr << "Error: listfile (" << filename << ") cannot be opened for reading" << endl;
				return -1; //error
			}
			return 1;
		} else {
			if(spacemulti){
				// space delimetered list of options
				int pos=0;
				while(options[0][pos]!='\0'){
					const char* tmparg=&options[0][pos];
					string q(tmparg);
					istringstream ostr(q);
					string s;
					ostr >> s;
					const char* t=s.c_str();
					//const char* t=tmparg;
					if((tmparg[0]=='@') && (!OptionList_trait<T>::external_AT)){ // sublist
						spacemulti=false;
						if(setvalue(&t,1)!=1){
							cerr << "Error in command line" << endl;
							cerr << "  while interpreting embeded list in position: " << tmparg << endl;
							return -1; //error
						}
						spacemulti=true;
					} else { // not a list or the option can handle @ sign by itself
						int ret=option.setvalue(&t, size); //FIXME: This will disallow further extention (StructOption)
						if(ret>0){
							newvalue(option);
							clearoption(option);
						} else
							return ret;
					}
					++pos;
					while((options[0][pos]!='\0') && (options[0][pos]!=' ')) ++pos;
					while((options[0][pos]!='\0') && (options[0][pos]==' ')) ++pos;
				}
				return 1;
			} else {
				int ret=option.setvalue(options, size);
				if(ret>0){
					newvalue(option);
					clearoption(option);
				}
				return ret;
			}
		}
	}
	return -1; //error
}


template<typename VT>  inline VT lgetmin(VT v1, VT v2){ return v1<v2 ? v1 : v2; }
template<typename VT>  inline VT lgetmax(VT v1, VT v2){ return v1>v2 ? v1 : v2; }
template<> std::string lgetmin(std::string v1, std::string v2){
	if(v1=="MAX") return v2;
	if(v2=="MAX") return v1;
	std::string::const_iterator i=v1.begin();
	std::string::iterator j=v2.begin(); // const does not work (gcc bug?)
	while( (i!=v1.end()) && (j!=v2.end()) && (*i==*j) ) { ++i; ++j; }
	return std::string(v2.begin(), j);
}
template<> std::string lgetmax(std::string v1, std::string v2){
	if(v1=="MIN") return v2;
	if(v2=="MIN") return v1;
	std::string::iterator i=v1.end();
	std::string::iterator j=v2.end(); // const does not work (gcc bug?)
	while( (i!=v1.begin()) && (j!=v2.begin()) && (*i==*j) ) { --i; --j; }
	if (*i!=*j) ++i;
	return std::string(i, v1.end());
}

template<typename T>
void Cmd_ns::OptionList<T>::newvalue(typename T::value_type v){
	values.push_back(v);
	min=lgetmin(v,min);
	max=lgetmax(v,max);
}

// functions
template<class IITER, class CONT>
void Cmd_ns::transformtocontainer(IITER beg, IITER end, CONT& opts, typename CONT::value_type::item_type init){
	while(beg!=end){
		if((std::string)(*beg)!=""){ // be compatible: skip empty lines
			opts.push_back(typename CONT::value_type(init));
			opts.back().setvalue(beg->c_str());
		}
		++beg;
	}
}

#ifndef OPTION_LIST_H
#define OPTION_LIST_H

#include <fstream>
#include <algorithm>
#include <vector>
#include <libCmd/CmdOption.h>

namespace Cmd_ns{

template<typename VT> struct OptionList_trait;
/*!
 * This enables the user to use a file in place of a given command line option.
 * the purpose of such option is to specify a list of options i.e. a list of image filenames
 * of a list of cluser ids, etc.
 * For the user this feature is transparent i.e. the option can still be used in the
 * original way (without listfile, just one single value).
 * The user must indicate the presence of a list file by starting the the option value
 * with an ``@'' sign. (no space between @ and the filename).
 * Optionally the user can specify a range of lines to read from the list file by
 * the ``:'' modifiers following the value. Two number (separated by ``:''_ can be
 * specified indicating the first line to read (line numbers are starting from 0) and
 * the number lines. If the second number is omited tha values are read till the end of the file.
 * E.g:
 * 	-OPTNAME @a.lst:3:6
 * will read the 6 values for OPTNAME from the file ``a.lst'' starting with the 4th line.
 * List files can contain embeded list files with the same (``@'') syntax.
 *
 * For the program this option always provides a list a values of the template type
 * option. the option type is accessible via item_type and must inherit from CmdOption.
 * E.g of declaration:
 * 	OptionList<UIntOption> cid(UIntOption("cid", "cluser identifiers", 0, UINT_MAX));
 * this declaration creates an list of unsigned int options for a program.
 * The user can pass a value for such option e.g:
 * 	(1)    -cid 3
 * 	(2)    -cid '3 5 7 1'
 * 	(3)    -cid @cidlist.txt
 * 	(4)    -cid @cidlist.txt:5
 * 	(5)    -cid @cidlist.txt:5:2
 * 	(6)    -cid '@cidlist.txt:0:3 @cidlist.txt:10:2'
 *
 * 	Automatic space splitting at command line (2) can be disabled by passing an additional
 * 	``false'' boolean parameter to the constructor of OptionList.
 * 	It is enabled as default for OptionLists that does not contain additional lists, for
 * 	OptionList<OptionList<...> > it is disabled.
 * 	The OptionList class internally stores a vector of item_type (options).
 * 	Iterators (begin(), end()) and operator[](int) and function size() are provided to
 * 	access the the options.
 * 	If you decide to use OptionList<OptionList<...> > you should be experienced with
 * 	the usage of such option. (You might also need to modify the default sm
 * 	values of the constructor.
 */


template<typename T> class OptionList;

template<typename VT>
struct OptionList_trait{
	static const bool external_AT=false;
	static const bool default_sm=true;
	static const bool haschild=false;
};


template<typename T>
struct OptionList_trait<OptionList<T> >{
	static const bool external_AT=true;
	static const bool default_sm=false;
	static const bool haschild=true;
};

template<typename T> class OptionList : public StringOption,
OptionList_trait<T>{

	public:

	typedef T item_type;
	typedef std::vector<typename T::value_type> value_type;
	//typedef item_type value_type;
	typedef typename std::vector<typename T::value_type>::const_iterator const_iterator;

	OptionList(T baseoption, bool sm=OptionList_trait<T>::default_sm);

	virtual int setvalue(const char ** options, int size);
	int setvalue(const char* so);
	//virtual void printvalue(std::ostream& out) const { out << ( value ? "true" : "false");};
	virtual void printshortformparam(std::ostream& out) const { option.printshortformparam(out);};

	//operator value_type () { return value; }
	
	typename std::vector<typename T::value_type>::const_iterator begin() const { return values.begin(); }
	typename std::vector<typename T::value_type>::const_iterator end() const { return values.end(); }
	size_t size() const { return values.size(); }
	typename T::value_type getmax() const { return max; }
	typename T::value_type getmin() const { return min; }
	bool haselement(const typename T::value_type& e) const { return (std::find(values.begin(), values.end(), e)!=values.end()); }
	bool haselement(const typename T::value_type& e, int begin, int end) const { return (std::find(&values[begin],&values[end],e)!=&values[end]); }

	const typename T::value_type& operator[](int i) const { return values[i]; }

	virtual value_type getvalues() const { return values; }

	operator value_type () const { return values; }

	void clear() { values.clear(); }

	//static const char LSTHELP[]="(with @, can be a listfile)";

		std::vector<typename T::value_type> values;
	private:

		void newvalue(typename T::value_type v);

		T option;
		typename T::value_type min;
		typename T::value_type max;
		bool spacemulti;
		bool magicspace; //! turns spacemulti on, if it was off

}; // OptionList

// helper-funciton
/*!
 * The existence transformtocontainer became OBSOLATE since now OptionLists can be
 * embeded to each other. We keep this function for backward compatibility or
 * for other purposes.
 * The the old workaround for such embeded situations is to declare an option is of StringOptions e.g.
 * 	OptionList<StringOption> mnumbersopt(StringOption("mnumbers","the multi numbers"),false);
 * Note: the ``false'' argument at the end!
 * and in the begining of the program call the following function:
 * 	transformtocontainer(mnumbersopt.begin(), mnumbersopt.end(), mnumbers, theoption);
 *
 * where theoption the a type of CmdOption and mnumbers is a container of theoption with
 * the support of push_back().
 * Than mnumbers can be seen as embeded OptionLists, e.g. can be browsed as:
 * (assuming IntOption for the type of theoption):
 * 		vector<OptionList<IntOption> >::const_iterator i=mnumbers.begin();
 * 		while(i!=mnumbers.end()){
 * 			OptionList<OptionList<IntOption> >::item_type::const_iterator j=i->begin();
 * 			while(j!=i->end()){
 * 				cout << (int)(*j) << endl;
 * 				++j;
 * 			}
 * 			cout << "----" << i->size() << "---" << endl;
 * 			++i;
 * 		}
 */
template<typename IITER, typename CONT>
inline void transformtocontainer(IITER beg, IITER end, CONT& opts, typename CONT::value_type::item_type init);

}; // Cmd_ns

#include "OptionList.hh"

#endif

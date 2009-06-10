#include "DotUtils.hpp"

#include <sstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

namespace dot {
using namespace std;

string 
dot_head(const string& _title) {
  string str;
  str += "digraph machine {\n";
  if(_title != "")
    str += "label=\"" + _title + "\";\nlabelloc=top;\n";
  return str;
}

string 
dot_foot() {
  return "};\n";
}

struct fix_dot_chr {
  void operator()(char& _chr) {
    if(_chr == ':')
      _chr = '_';
    if(_chr == '.')
      _chr = '_';
    if(_chr == '-')
      _chr = '_';
  }
};

template<class T, class F>
inline
void for_all(T& _container, const F& _functor) {
  if(!_container.empty())
    for_each(_container.begin(),_container.end(),_functor);
}

/// removes things that are not allowed in a dot node id
string fix_dot_id(const string& _str) {
  string str = "N" + _str;
  for_all(str,fix_dot_chr());
  return str;
}


string
dot_node(const string& _id, 
	 const string& _label, 
	 const string& _shape,
	 const string& _properties) {
  string str;
  str += fix_dot_id(_id);
  str += " [shape = " + _shape;
  if(_label != "")
    str += ",label=\"" + _label + "\"";
  if(_properties != "")
    str += "," + _properties;
  str += "];\n";
  return str;
}



string dot_arrow( const string& _from, 
		  const string& _to, 
		  const string& _type, 
		  const string& _label,
		  const string& _properties) {
  string str;
  
  str += fix_dot_id(_from) + " " + _type + " " + fix_dot_id(_to); 
  
  if(_label != "" || _properties != "") {
    str += " [";
    if(_label != "")
      str +="label=\"" + _label + "\"";
    if(_properties != "") {
      if(_label != "") {
	str += ",";
      }
      str += _properties;
    }
    str += "]";
  }
  str += ";\n";
  return str;
}

string boxed_dot_arrow( const string& _from, 
			const string& _to, 
			const string& _type, 
			const string& _label,
			const string& _properties,
			const string& _box_type) {
  stringstream str;
  string shared_id = _from + "_" + _to;
  str << dot_arrow(_from, shared_id, _type, "",_properties);
  str << dot_node(shared_id, _label, _box_type);
  str << dot_arrow(shared_id, _to, _type, "",_properties);
  return str.str();
}

} // namespace dot

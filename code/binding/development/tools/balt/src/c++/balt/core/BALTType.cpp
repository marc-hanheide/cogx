#include "BALTType.hpp"

#include <balt/core/BALTException.hpp>


using namespace std;

TypeMap<string>::map BALTType::m_typeMap;

std::string BALTType::cfilt(const char* _typename) {

  //HACK: this is all really really ugly, cleanup later
  FILE *fpipe;

  std::string command = std::string(__CFILT__) + " " + _typename;
  //cout<<"command: "<<command<<endl;
  char line[256];
  vector<string> lines;

  if ( !(fpipe = (FILE*)popen(command.c_str(),"r")) )    {  // If fpipe is NULL
    throw (BALTException(__HERE__,"Unable to open pipe"));      
  }

  while ( fgets( line, sizeof line, fpipe)) {
    lines.push_back(line);
  }
  pclose(fpipe);

  //remove the trail \n
  string type = lines[0];
  type = type.erase(type.length() -1 );
  //clean up

  //cout<<"type: "<<type<<endl;

  return type;
}

const std::string& 
BALTType::demangle(const type_info& _type) {
  // string specialization:
  //if(_type == typeid(std::string)) {
  //    static string string_id("string");
  //return string_id;
    //}
  TypeMap<string>::map::iterator i = m_typeMap.find(&_type);
  if(i == m_typeMap.end()) {
    //cout<<"run c++filt"<<endl;
    string filt(cfilt(_type.name()));
    i = m_typeMap.insert(make_pair(&_type,filt)).first;
  }
  return i->second;
}


template <>
const std::string& BALTType::typeName<std::string>() {
  static string string_id("string");
  return string_id;
}

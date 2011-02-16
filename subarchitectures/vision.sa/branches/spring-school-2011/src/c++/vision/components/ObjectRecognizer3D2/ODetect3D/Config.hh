/**
 * $Id: Config.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#ifndef P_CONFIG_HH
#define P_CONFIG_HH

#include <stdlib.h>
#include <string>
#include <map>
#include "PNamespace.hh"

namespace P 
{

class Config
{
public:
  map<string, string> items;

public:
  Config() {}
  Config(const char *filename);
  void Load(const char *filename);
  void AddItem(const string &name, const string &value) {items[name] = value;}
  bool ExistItem(const string &name);
  string GetValueString(const string &name) {return items[name];}
  int GetValueInt(const string &name) {return atoi(items[name].c_str());}
  double GetValueDouble(const string &name) {return atof(items[name].c_str());}
};

}

#endif


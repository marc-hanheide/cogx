/**
 * $Id: Config.hh,v 1.4 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_CONFIG_HH
#define Z_CONFIG_HH

#include <stdlib.h>
#include <string>
#include <map>
#include "Namespace.hh"

namespace Z
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
  string GetValueString(const string &name) {return items[name];}
  int GetValueInt(const string &name) {return atoi(items[name].c_str());}
  double GetValueDouble(const string &name) {return atof(items[name].c_str());}
};

}

#endif


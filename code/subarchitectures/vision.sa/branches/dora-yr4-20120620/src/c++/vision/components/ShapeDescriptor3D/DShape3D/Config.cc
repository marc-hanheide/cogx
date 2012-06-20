/**
 * $Id$
 */

#include "ConfigFile.hh"
#include "Config.hh"

namespace P 
{

Config::Config(const char *filename)
{
  Load(filename);
}

void Config::Load(const char *filename)
{
  ConfigFile file(filename);
  string line, name, value;
  string::size_type pos;
  while(file.GetLine(line))
  {
    pos = 0;
    GetWord(line, name, pos);
    GetWord(line, value, pos);
    items[name] = value;
  }
}


bool Config::ExistItem(const string &name)
{
  map<string,string>::iterator it = items.find(name);
  if( it == items.end() ) return false;
  return true; 
}

}


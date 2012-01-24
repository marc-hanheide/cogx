/**
 * $Id: Config.cc,v 1.4 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include "ConfigFile.hh"
#include "Config.hh"

namespace Z
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

}


// ==================================================================
// Place.SA - Place Classification Subarchitecture
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of Place.SA.
//
// Place.SA is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Place.SA is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Place.SA. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * ConfigFile class.
 * \file ConfigFile.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-09
 */

#include "ConfigFile.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>

using namespace categorical;
using namespace std;
using namespace boost;


// ------------------------------------------------------
bool ConfigFile::isGroup(const std::string &line) const
{
  return starts_with(line, "[") && ends_with(line, "]");
}


// ------------------------------------------------------
bool ConfigFile::isItem(const std::string &line) const
{
  return contains(line, "=");
}


// ------------------------------------------------------
bool ConfigFile::isComment(const std::string &line) const
{
  return starts_with(line,"#");
}


// ------------------------------------------------------
void ConfigFile::parseGroup(const std::string &line, std::string &name) const
{
  name = string(line.begin()+1, line.end()-1);
  trim(name);
}


// ------------------------------------------------------
void ConfigFile::parseItem(const std::string &line, std::string &name, std::string &value) const
{
  iterator_range<string::const_iterator> i=find_first(line, "=");
  name=string(line.begin(), i.begin());
  trim(name);
  value=string(i.end(), line.end());
  trim(value);
}


// ------------------------------------------------------
bool ConfigFile::read(const std::string &filePath)
{
  ifstream file(filePath.c_str());
  if (!file.is_open())
    return false;

  string group="";
  while(!file.eof())
  {
    string line;
    getline(file, line);
    trim(line);
    if ((!line.empty()) && (!isComment(line)))
    {
      if (isGroup(line))
      {
        parseGroup(line, group);
        to_lower(group);
      }
      else if (isItem(line))
      {
        string name;
        string value;
        parseItem(line, name, value);
        to_lower(name);
        _map[group][name]=value;
      }
      else
      {
        file.close();
        return false;
      }
    }
  }

  file.close();
  return true;
}


// ------------------------------------------------------
bool ConfigFile::write(const std::string &filePath)
{
  ofstream file(filePath.c_str());
  if (!file.is_open())
    return false;

  bool first=true;

  // Write the empty group
  GroupMapType::iterator g = _map.find("");
  if (g!=_map.end())
  {
    first=false;

    for(ItemMapType::iterator j = g->second.begin(); j!=g->second.end(); ++j)
    {
      file<<j->first<<" = "<<j->second<<endl;
    }
  }

  // Write other groups
  for(GroupMapType::iterator i = _map.begin(); i!=_map.end(); ++i)
  {
    if (!i->first.empty())
    {
      if (first)
        file<<"["+i->first+"]"<<endl;
      else
        file<<"\n["+i->first+"]"<<endl;
      first=false;

      for(ItemMapType::iterator j = i->second.begin(); j!=i->second.end(); ++j)
      {
        file<<j->first<<" = "<<j->second<<endl;
      }
    }
  }

  file.close();
  return true;
}


// ------------------------------------------------------
const std::string &ConfigFile::getStrValue(const std::string &group, const std::string &name, const std::string &def) const
{
  GroupMapType::const_iterator g = _map.find(to_lower_copy(group));
  if (g==_map.end())
    return def;
  else
  {
    ItemMapType::const_iterator i = g->second.find(to_lower_copy(name));
    if (i==g->second.end())
      return def;
    else
      return i->second;
  }
}


// ------------------------------------------------------
int ConfigFile::getIntValue(const std::string &group, const std::string &name, int def) const
{
  GroupMapType::const_iterator g = _map.find(to_lower_copy(group));
  if (g==_map.end())
    return def;
  else
  {
    ItemMapType::const_iterator i = g->second.find(to_lower_copy(name));
    if (i==g->second.end())
      return def;
    else
      return lexical_cast<int>(i->second);
  }
  return true;
}


// ------------------------------------------------------
double ConfigFile::getDoubleValue(const std::string &group, const std::string &name, double def) const
{
  GroupMapType::const_iterator g = _map.find(to_lower_copy(group));
  if (g==_map.end())
    return def;
  else
  {
    ItemMapType::const_iterator i = g->second.find(to_lower_copy(name));
    if (i==g->second.end())
      return def;
    else
      return lexical_cast<double>(i->second);
  }
  return true;
}


// ------------------------------------------------------
bool ConfigFile::getBoolValue(const std::string &group, const std::string &name, bool def) const
{
  GroupMapType::const_iterator g = _map.find(to_lower_copy(group));
  if (g==_map.end())
    return def;
  else
  {
    ItemMapType::const_iterator i = g->second.find(to_lower_copy(name));
    if (i==g->second.end())
      return def;
    else
    {
      string val=i->second;
      to_lower(val);
      if (val=="false")
        return false;
      else if (val=="true")
        return true;
      else 
        throw bad_lexical_cast();
    }
  }
  return true;
}


// ------------------------------------------------------
void ConfigFile::setValue(const std::string &group, const std::string &name, const std::string &value)
{
  _map[group][name]=value;
}


// ------------------------------------------------------
void ConfigFile::setValue(const std::string &group, const std::string &name, bool value)
{
  if (value)
    _map[group][name]="True";
  else
    _map[group][name]="False";
}


// ------------------------------------------------------
void ConfigFile::setValue(const std::string &group, const std::string &name, const boost::filesystem::path &value)
{
  _map[group][name]=value.string();
}


// ------------------------------------------------------
void ConfigFile::setValue(const std::string &group, const std::string &name, long value)
{
  _map[group][name]=lexical_cast<string>(value);
}


// ------------------------------------------------------
void ConfigFile::setValue(const std::string &group, const std::string &name, int value)
{
  _map[group][name]=lexical_cast<string>(value);
}



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
 * LabelFile class.
 * \file LabelFile.cpp
 * \author Andrzej Pronobis
 * \date 2008-08-15
 */


#include "LabelFile.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iostream>

using namespace categorical;
using namespace std;
using namespace boost;


// ------------------------------------------------------
bool LabelFile::isComment(const std::string &line) const
{
  return starts_with(line,"#");
}

// ------------------------------------------------------
bool LabelFile::read(const std::string &filePath)
{
  ifstream file(filePath.c_str());
  if (!file.is_open())
  {
    cerr<<"LabelFile: Cannot open file '"<<filePath<<"'."<<endl;
    return false;
  }

  while(!file.eof())
  {
    // Get line
    string line;
    getline(file, line);
    trim(line);

    if ((line.empty()) || (isComment(line)))
      continue;

    // Split line
    vector<string> tokens;
    split(tokens, line, is_any_of(" \t") );
    if (tokens.size()<2)
    {
      cerr<<"LabelFile: Incorrect line '"+line+"'. Lines must contain number and name."<<endl;
      file.close();
      return false;
    }
    trim(tokens[0]);
    trim(tokens[1]);

    // Get values
    Label l;
    try
    {
      l.number = lexical_cast<int>(tokens[0]);
    }
    catch (bad_lexical_cast)
    {
      file.close();
      return false;
    }
    if (tokens[1].empty())
    {
      file.close();
      return false;
    }
    l.name=tokens[1];

    // Check if already in the list
    if ((isLabelNo(l.number)) || (isLabelName(l.name)))
    {
      file.close();
      return false;
    }

    _labels.push_back(l);
  }

  file.close();

  // Check if empty
  if (_labels.empty())
    return false;
  else
    return true;
}


// ------------------------------------------------------
std::string LabelFile::labelNoToName(int number) const
{
  for (list<Label>::const_iterator i=_labels.begin(); i!=_labels.end(); i++)
  {
    if (i->number==number)
      return i->name;
  }

  return "";
}


// ------------------------------------------------------
bool LabelFile::isLabelName(std::string name) const
{
  for (list<Label>::const_iterator i=_labels.begin(); i!=_labels.end(); i++)
  {
    if (i->name==name)
      return true;
  }
  return false;
}


// ------------------------------------------------------
bool LabelFile::isLabelNo(int number) const
{
  for (list<Label>::const_iterator i=_labels.begin(); i!=_labels.end(); i++)
  {
    if (i->number==number)
      return true;
  }
  return false;
}



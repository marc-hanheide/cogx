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
 * \file LabelFile.h
 * \author Andrzej Pronobis
 * \date 2008-08-15
 */


#ifndef __PLACE_LABEL_FILE__
#define __PLACE_LABEL_FILE__

#include <string>
#include <list>
#include "boost/filesystem.hpp" 


namespace categorical
{


/**
 * Provides access to files storing class labels.
 */
class LabelFile
{
public:

  struct Label
  {
    int number;
    std::string name;
  };

  typedef std::list<Label>::const_iterator const_iterator;


public:

  /** Constructor. */
  LabelFile()
  {}

  /** Reads label file. If error occurs, returns false. */
  bool read(const std::string &filePath);

  int getLabelCount() const
  { return _labels.size(); }

  std::string labelNoToName(int number) const;

  bool isLabelName(std::string name) const;

  bool isLabelNo(int number) const;

  const_iterator begin() const
  { return _labels.begin(); }

  const_iterator end() const
  { return _labels.end(); }

  void clear()
  { _labels.clear(); }


private:

  bool isComment(const std::string &line) const;


private:

  std::list<Label> _labels;



};

}


#endif



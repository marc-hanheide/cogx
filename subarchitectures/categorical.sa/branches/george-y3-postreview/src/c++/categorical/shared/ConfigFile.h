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
 * \file ConfigFile.h
 * \author Andrzej Pronobis
 * \date 2008-08-09
 */


#ifndef __PLACE_CONFIG_FILE__
#define __PLACE_CONFIG_FILE__

#include <string>
#include <map>
#include "boost/filesystem.hpp" 


namespace categorical
{

  /**
   * Provides access to configuration stored in config files.
   * Uses the ini-like format of the config file.
   */
  class ConfigFile
  {
  public:

    /** Constructor. */
    ConfigFile()
    {}

    /** Reads configuration from a file. If error occurs, returns false. */
    bool read(const std::string &filePath);

    /** Writes configuration to a file. If error occurs, returns false. */
    bool write(const std::string &filePath);

    /** Returns a value of a string item identified by name and group.
     *  If the item cannot be found, default value is used. */
    const std::string &getStrValue(const std::string &group, const std::string &name, const std::string &def=std::string()) const;

    /** Returns a value of an integer item identified by name and group.
     * If the item cannot be found, default value is used.
     * Throws bad_lexical_cast if conversion error occurs.*/
    int getIntValue(const std::string &group, const std::string &name, int def=0) const;

    /** Returns a value of a double item identified by name and group.
     * If the item cannot be found, default value is used.
     * Throws bad_lexical_cast if conversion error occurs.*/
    double getDoubleValue(const std::string &group, const std::string &name, double def=0.0) const;

    /** Returns a value of a boolean item identified by name and group.
     * If the item cannot be found, default value is used.
     * Throws bad_lexical_cast if conversion error occurs.*/
    bool getBoolValue(const std::string &group, const std::string &name, bool def=false) const;

    /** Adds item=value identified by name and group. */
    void setValue(const std::string &group, const std::string &name, const std::string &value);

    /** Adds item=value identified by name and group. */
    void setValue(const std::string &group, const std::string &name, bool value);

    /** Adds item=value identified by name and group. */
    void setValue(const std::string &group, const std::string &name, long value);

    /** Adds item=value identified by name and group. */
    void setValue(const std::string &group, const std::string &name, int value);

    /** Adds item=value identified by name and group. */
    void setValue(const std::string &group, const std::string &name, const boost::filesystem::path &value);

  private:

    bool isGroup(const std::string &line) const;
    bool isItem(const std::string &line) const;
    bool isComment(const std::string &line) const;
    void parseGroup(const std::string &line, std::string &name) const;
    void parseItem(const std::string &line, std::string &name, std::string &value) const;

    typedef std::map<std::string, std::string> ItemMapType;
    typedef std::map<std::string, ItemMapType> GroupMapType;

    /** Map of variables. */
    GroupMapType _map;
  };

}


#endif

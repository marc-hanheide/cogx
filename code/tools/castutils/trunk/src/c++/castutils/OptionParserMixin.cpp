/**
 * Author: Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 * Date:   27.03.2012
 **/

#include "OptionParserMixin.hpp"

#include <boost/filesystem.hpp>

using namespace std;
using namespace boost;


namespace castutils {


// ------------------------------------------------------
string OptionParserMixin::parseOption(
    const string name,
    const string defaultValue) const
{
  checkConfigSet();
  return parseOption(_config, name, defaultValue);
}


// ------------------------------------------------------
string OptionParserMixin::parseOption(
    const map<string, string> &config,
    const string name,
    const string defaultValue) const
{
  map<string, string>::const_iterator it = config.find(name);

  if (it != config.end())
    return it->second;

  return defaultValue; // default if not given
}


// ------------------------------------------------------
bool OptionParserMixin::parseOptionFlag(
    const string name) const
{
  checkConfigSet();
  return parseOptionFlag(_config, name);
}


// ------------------------------------------------------
bool OptionParserMixin::parseOptionFlag(
    const map<string, string> &config,
    const string name) const
{
  map<string, string>::const_iterator it = config.find(name);

  if (it != config.end())
  {
    if (it->second == "true")
    {
      return true;
    }
    else if (it->second == "false")
    {
      return false;
    }
    else
    {
      _component->error("Invalid value '%s' for flag option '%s'. "
                        "Should be one of {'true', 'false'}",
                        it->second.c_str(), name.c_str());

      return false; // default if malformed
    }
  }

  return false; // default if not given
}


// ------------------------------------------------------
string OptionParserMixin::parseOptionPath(
    const string name,
    const string defaultValue) const
{
  checkConfigSet();
  return parseOptionPath(_config, name, defaultValue);
}


// ------------------------------------------------------
string OptionParserMixin::parseOptionPath(
    const map<string, string> &config,
    const string name,
    const string defaultValue) const
{
  filesystem::path tmp = parseOption(config, name, defaultValue);
  try
  {
    tmp = filesystem::system_complete(tmp);
  }
  catch (filesystem::filesystem_error &e)
  {
    _component->error("Value '%s' of option '%s' could not be resolved as a "
                      "valid path: ", tmp.c_str(), name.c_str(), e.what());
    return "";
  }

  return tmp.native();
}


// ------------------------------------------------------
void OptionParserMixin::setConfig(const map<string, string> &config)
{
  _config = config;
  _configSet = true;
}


// ------------------------------------------------------
void OptionParserMixin::checkConfigSet() const
{
  if (!_configSet)
  {
    _component->error("OptionParserMixin: Accessing saved config "
                      "config, but no config set");
  }
}


} // namespace castutil

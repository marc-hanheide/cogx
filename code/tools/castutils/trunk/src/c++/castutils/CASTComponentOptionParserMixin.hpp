/**
 * Author: Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 * Date:   27.03.2012
 **/

#ifndef __CASTUTILS_CASTOPTIONPARSERMIXIN_HPP__
#define __CASTUTILS_CASTOPTIONPARSERMIXIN_HPP__

#include <cast/core/CASTComponent.hpp>

#include <boost/lexical_cast.hpp>


namespace castutils {


class CASTComponentOptionParserMixin
{
public:
  
  CASTComponentOptionParserMixin(cast::CASTComponent const* const component):
      _component(component)
  {
  }

  virtual ~CASTComponentOptionParserMixin() 
  {
  }

protected:

  /** Parse a string option.
   *
   * The default value is returned if the option was not passed.
   */
  std::string parseOption(
      const std::map<std::string, std::string> &config,
      const std::string name, 
      const std::string defaultValue) const;


  /** Parse a string option. Use saved config.
   */
  std::string parseOption(
      const std::string name, 
      const std::string defaultValue) const;


  /** Parse a flag (i.e. boolean) option.
   *
   * Default value is 'false'. It is returned if the option was not passed. If
   * the option was passed, but the value passed is maleformed (i.e. not "true"
   * or "false"), print error and return default value.
   */
  bool parseOptionFlag(
      const std::map<std::string, std::string> &config,
      const std::string name) const;


  /** Parse a flag (i.e. boolean) option. Use saved config.
   */
  bool parseOptionFlag(
      const std::string name) const;
      

  /** Parse an option and perform a lexical cast.
   *
   * The default value is returned if the option was not passed. If the option
   * was passed, but the lexical cast throws an exception, print error and
   * return default value.
   */
  template<class T>
  T parseOptionLexicalCast(
      const std::map<std::string, std::string> &config,
      const std::string name,
      const T defaultValue) const
  {
    std::map<std::string, std::string>::const_iterator it = config.find(name);
    
    if (it != config.end())
    {
      try
      {
        return boost::lexical_cast<T>(it->second);
      }
      catch(boost::bad_lexical_cast &)
      {
        _component->error("Value '%s' of option '%s' is not of type '%s'", 
                          it->second.c_str(), name.c_str(), typeid(T).name());
        
        return defaultValue; // default if malformed
      }
    }
    
    return defaultValue; // default if not given
  }


  /** Parse an option and perform a lexical cast. Use saved config.
   */
  template<class T>
  T parseOptionLexicalCast(
      const std::string name,
      const T defaultValue) const
  {
    return parseOptionLexicalCast<T>(_config, name, defaultValue);
  }


  /** Parse a path option and resolve the path.
   *
   * If the option was not passed, the default value is resolved and
   * returned. If in that case the default value can not be resolved, print an
   * error and return an empty string. If the option was passed, but the value
   * could not be resolved as a path, print error and return default value, else
   * return the resolved path.
   */ 
  std::string parseOptionPath(
      const std::map<std::string, std::string> &config,
      const std::string name, 
      const std::string defaultValue) const;


  /** Parse a path option and resolve the path. Use saved config.
   */
  std::string parseOptionPath(
      const std::string name, 
      const std::string defaultValue) const;


protected:

  void setConfig(const std::map<std::string, std::string> &config);

  std::map<std::string, std::string> _config;

private:

  cast::CASTComponent const* const _component;

};


} // namespace castutils

#endif

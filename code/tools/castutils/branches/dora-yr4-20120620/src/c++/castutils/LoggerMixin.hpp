/**
 * Author: Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 * Date:   10.04.2012
 **/

#ifndef __CASTUTILS_LOGGERMIXIN_HPP__
#define __CASTUTILS_LOGGERMIXIN_HPP__

#include <cast/core/CASTComponent.hpp>


namespace castutils {

/**
 * Provide some extra convience functions to for logging in cast components. See
 * the ...Baseclass variant for extra convenience.
 *
 * EXAMPLE: If you have a component like this (The base class 'ManagedComponent'
 * is just exemplar. Could be any CASTComponent derived class):
 *
 * class MyFunkyComponent: public ManagedComponent { ... };
 *
 * Use this declaration instead:
 *
 * class MyFunkyComponent:
 *     public ManagedComponent,
 *     public LoggerMixin
 * {
 * public:
 *   MyFunkyComponent():
 *       LoggerMixin(this)
 *   { ... }
 *
 * ...
 * };
 */
class LoggerMixin
{
public:

  LoggerMixin(cast::CASTComponent * const component):
      _component(component)
  {
  }

  virtual ~LoggerMixin()
  {
  }

public:

  /**
   * Print out the input in a formatted way to warn log.
   */
  virtual void warn(const std::string &s);


  /**
   * printf-like method for warn log.
   */
  virtual void warn(const char *format, ...);


  /**
   * Print out the input in a formatted way to fatal log.
   */
  virtual void fatal(const std::string &s);


  /**
   * printf-like method for fatal log.
   */
  virtual void fatal(const char *format, ...);


  /**
   * Log an error if condition is not true.
   */
  virtual void cast_assert(const bool condition, const std::string &s);


  /**
   * printf-like method for cast assertions.
   */
  virtual void cast_assert(const bool condition, const char *format, ...);

private:

  cast::CASTComponent * const _component;

};


/**
 * Use this for extra convenience. Specify the class you would normally have
 * inherited from (a CASTComponent derived class) as the template parameter. The
 * advantage is that you don't have to touch the constructor.
 *
 * EXAMPLE: If you have a component like this (The base class 'ManagedComponent'
 * is just exemplar. Could be any CASTComponent derived class):
 *
 * class MyFunkyComponent: public ManagedComponent { ... };
 *
 * Use this declaration instead:
 *
 * class MyFunkyComponent:
 *     public LoggerMixinBaseclass<ManagedComponent>
 * { ... };
 */
template <class CASTComponentClass>
class LoggerMixinBaseclass:
    public CASTComponentClass,
    public LoggerMixin
{
public:
  LoggerMixinBaseclass():
      LoggerMixin(this)
  {
  }
};


} // namespace castutils

#endif

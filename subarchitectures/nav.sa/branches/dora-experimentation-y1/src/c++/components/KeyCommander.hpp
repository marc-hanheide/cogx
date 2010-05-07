//
// = FILENAME
//    
//
// = FUNCTION
//
// = AUTHOR(S)
//    Dorian Galvez Lopez
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Dorian Galvez Lopez
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef KeyCommander_hpp
#define KeyCommander_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <cast/architecture/WorkingMemoryReaderComponent.hpp>
#include <NavData.hpp>

namespace navsa {

/**
 * This class prvides a rudimentary key clicking interface to control
 * the robot. There seems to be some issues with it in the current form
 * of the system though. After one of the updates it no longer works.
 *
 * @param -u sleep between reading the key
 * @param --fixed-prio specify the priority for all following commands, do not need to specify it
 * @param --no-key no listening to the keys
 *
 * @author Patric Jensfelt
 * @see
 */
class KeyCommander: public cast::ManagedComponent
{
public:
  KeyCommander();
  virtual ~KeyCommander(){}
  virtual void start();
  virtual void runComponent();

protected:

  void configure(const std::map<std::string,std::string> &_config);

private:

  void owtNavCommand(const cast::cdl::WorkingMemoryChange &objID);

  bool m_CanAbort;
  long m_USleep;
  int m_AutomaticCommands;
  std::string m_FixedPrio;
  bool m_NoKey;
  long m_InitSleep; // ms
  
  void cancelObjCommand(const std::string &id, const std::string &sa);
};

}; // namespace navsa

#endif  // KeyCommander_hpp

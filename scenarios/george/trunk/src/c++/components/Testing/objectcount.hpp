/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_OBJECTCOUNT_HPP_4F7C3CBC_
#define _TESTING_OBJECTCOUNT_HPP_4F7C3CBC_

#include "castmachine.hpp"

namespace testing
{

class CObjectCountTest // This is a state-machine factory
{
  CStatePtr stStart(CCastMachine*);
  CStatePtr stTableEmpty(CCastMachine*);
  CStatePtr stWaitToAppear(CCastMachine*);
public:
  CMachinePtr init(); // TODO: better factory interface
};

} //namespace
#endif /* _TESTING_OBJECTCOUNT_HPP_4F7C3CBC_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim

/**
 * @author Marko Mahnič
 * @created April 2012
 */
#ifndef _TESTING_Y4LEARNING_HPP_4F8C0214_
#define _TESTING_Y4LEARNING_HPP_4F8C0214_

#include "castmachine.hpp"

namespace testing
{

class CY4Learning // This is a state-machine factory
{
  CStatePtr stStart(CCastMachine*);
  CStatePtr stTableEmpty(CCastMachine*);
  CStatePtr stWaitToAppear(CCastMachine*);
  CStatePtr stTeach(CCastMachine* pMachine);
  long mSceneId;
  long mTeachingStep;
  long mStepsTaught;
public:
  CMachinePtr init(); // TODO: better factory interface
};

} // namespace
#endif /* _TESTING_Y4LEARNING_HPP_4F8C0214_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim

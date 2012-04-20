/**
 * @author Marko Mahnič
 * @created April 2012
 */
#include "tester.hpp"

extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new testing::CTester();
  }
}

#include "objectcount.hpp"

namespace testing { 

template<typename ResultT, typename IterableT, typename UnaryFuncT>
ResultT min_value(const IterableT& list, UnaryFuncT op) {
  auto b = list.begin();
  auto e = list.end();
  assert (b != e);
  ResultT r = op(*b);
  for (++b; b != e; ++b) {
    ResultT v = op(*b);
    if (v < r) r = v;
  }
  return r;
}

template<typename ResultT, typename IterableT>
ResultT min_value(const IterableT& list) {
  auto b = list.begin();
  auto e = list.end();
  assert (b != e);
  ResultT r = *b;
  for (++b; b != e; ++b) {
    ResultT v = *b;
    if (v < r) r = v;
  }
  return r;
}

void CTester::runComponent()
{
  std::vector<std::string> mEvents;
  std::vector<CMachinePtr> machines;
  CObjectCountTest t;
  machines.push_back(t.init());

  while (isRunning()) {
    if (mEvents.size()) {
      for (auto m : machines) {
        m->checkEvents(mEvents);
      }
    }

    // See if you can put the thread to sleep (time-to-sleep)
    long tts = min_value<long>(machines, [](CMachinePtr m) -> long {
        if (m->isWaitingForEvent()) {
          return 1;
        }
        if (m->isFinished()) {
          return 20;
        }
        return m->timeToSleep();
      });
    if (tts > 0) {
      sleepComponent(tts);
      if (mEvents.size()) {
        for (auto m : machines) {
          m->checkEvents(mEvents);
        }
      }
    }
    for (auto m : machines) {
      m->runOneStep();
    }
  }
}

} //namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim

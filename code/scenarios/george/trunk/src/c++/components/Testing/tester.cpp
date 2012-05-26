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

#include "y4learning.hpp"

#include <castutils/Timers.hpp>
#include <sstream>
#include <climits>

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

void CTester::configure(const std::map<std::string,std::string> & _config)
{
  std::map<std::string,std::string>::const_iterator it;
  CCastComponentMixin* pCastMix;
  CMachinePtr pMachine;
#ifdef FEAT_VISUALIZATION
  mDisplay.configureDisplayClient(_config);
#endif

  CY4Learning t;
  pMachine = t.createMachine(this);
  pCastMix = dynamic_cast<CCastComponentMixin*>(&*pMachine);
  if (pCastMix) {
    pCastMix->configure(_config);
  }
  mMachines.push_back(pMachine);
}

void CTester::start()
{
  for (auto pm : mMachines) {
    auto pCastMix = dynamic_cast<CCastComponentMixin*>(&*pm);
    if (pCastMix) {
      pCastMix->start();
    }
  }

#ifdef FEAT_VISUALIZATION
  mDisplay.connectIceClient(*this);
#endif
}

std::string mkseqid_rev(const std::string& prefix, long id)
{
  std::ostringstream ss;
  ss << (LONG_MAX - id);
  std::string sid = ss.str();

  while (sid.length() < 8) {
    sid = "0" + sid;
  }

  return prefix + sid;
}

void CTester::runComponent()
{
  time_t t = time(0);
  tm *curTime = localtime(&t);
  char cbuf[32];

#ifdef FEAT_VISUALIZATION
  const long nDisplay = 100;
  const long nRemoveBatch = nDisplay * 0.2;
  const std::string sViewName("Evaluator");

  std::vector<long> lastReport;
  std::vector<long> removedReport; // reports that were already removed
  lastReport.resize(mMachines.size());
  removedReport.resize(mMachines.size());
  for (long& r : lastReport) r = 0;
  for (long& r : removedReport) r = 0;
  castutils::CMilliTimer tmSend;
  tmSend.restart();

  strftime(cbuf, 32, "Start time: %H:%M:%S", curTime);
  mDisplay.setHtml(sViewName, "009", cbuf);
  mDisplay.setHtml(sViewName, "010", "<table>");
  mDisplay.setHtml(sViewName, "500", "</table>");
#endif

  while (isRunning()) {
    for (auto pm : mMachines) {
      pm->checkEvents();
    }

    // See if you can put the thread to sleep (time-to-sleep)
    long tts = min_value<long>(mMachines, [](CMachinePtr pm) -> long {
        if (pm->isWaitingForEvent()) {
          return 1;
        }
        if (pm->isFinished()) {
          return 20;
        }
        return pm->timeToSleep();
      });
    if (tts > 0) {
      sleepComponent(tts);
      for (auto pm : mMachines) {
        pm->checkEvents();
      }
    }
    std::ostringstream ss;
    for (auto pm : mMachines) {
      try {
        pm->runOneStep();
      }
      catch (...) {
        error("runOneStep failed");
      }
    }
#ifdef FEAT_VISUALIZATION
    for (int i = 0; i < mMachines.size(); i++) {
      auto& pm = mMachines[i];
      long n = pm->getStepNumber();
      if (n == lastReport[i] && !pm->isSwitching() && tmSend.elapsed() < 2000) {
        continue;
      }
      bool bNewState = (n != lastReport[i]);
      lastReport[i] = n;
      tmSend.restart();
      if (removedReport[i] < n - nDisplay) {
        for (int j = 0; j < nRemoveBatch; j++) {
          mDisplay.removePart(sViewName, mkseqid_rev("011-", removedReport[i]));
          ++removedReport[i];
        }
      }

      t = time(0);
      curTime = localtime(&t);

      ss.str("");
      strftime(cbuf, 32, "%H:%M:%S", curTime);
      ss << "<tr><td>" << n 
        << "</td><td>" << cbuf // curTime
        << "</td><td>";
      pm->writeStateDescription(ss);
      ss << "</td></tr>\n";
      mDisplay.setHtml(sViewName, mkseqid_rev("011-", n), ss.str());
      if (bNewState) {
        log("%s", ss.str().c_str());
      }

      ss.str("");
      pm->writeMachineDescription(ss);
      ss << "<hr>\n";
      mDisplay.setHtml(sViewName, "005", ss.str());
    }
#endif
  }
}

} //namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et ft=cpp11.cpp :vim

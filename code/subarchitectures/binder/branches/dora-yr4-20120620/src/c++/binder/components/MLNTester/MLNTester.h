/**
 * @author Alen Vrecko
 * @date May 2011
 *
 * Markov logic network engine tester.
 */

#ifndef MLN_TESTER_H
#define MLN_TESTER_H

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cast/architecture/ManagedComponent.hpp>

#include <beliefs_cast.hpp>
#include <binder.hpp>
#include <ref.hpp>

using namespace de::dfki::lt::tr::beliefs::slice;
using namespace de::dfki::lt::tr::dialogue::ref;

namespace cast
{

class MLNTester :  public ManagedComponent

{
 private:
  bool doDisplay;
  
  std::string m_inferenceString;
  std::string m_id;
  std::string m_resultWMId;
  /**
   * Name of the binder subarchitecture
   */
  std::string m_bindingSA;
  
  
   std::queue<org::cognitivesystems::binder::mln::InferredResultPtr> m_resultQueue; 
  
  /**
   * callback function called whenever there is new evidence
   */
  void newInferredResult(const cdl::WorkingMemoryChange & _wmc);
  
  void queueNewInferredResult(org::cognitivesystems::binder::mln::InferredResultPtr res);
  

 protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();

 public:
  virtual ~MLNTester() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

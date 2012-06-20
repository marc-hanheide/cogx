/**
 * @author Alen Vrecko
 * @date September 2011
 *
 * A component that filters beliefs and provides evidence to a Markov logic network engine.
 */

#ifndef MLN_REF_RESOLUTION_CLIENT_H
#define MLN_REF_RESOLUTION_CLIENT_H

#include <AbsMLNClient.h>
#include <ref.hpp>
#include <beliefs.hpp>
#include <beliefs_cast.hpp>

using namespace de::dfki::lt::tr::dialogue::ref;
using namespace de::dfki::lt::tr::beliefs::slice;
using namespace de::dfki::lt::tr::beliefs::slice::sitbeliefs;
using namespace de::dfki::lt::tr::beliefs::slice::logicalcontent;

namespace cast
{

class MLNRefResolutionClient : public AbsMLNClient
{
 private:
  cast::cdl::WorkingMemoryAddress m_constraintAddr;
  std::vector<std::string> m_currentConstraints;
//  std::vector<std::string> m_evdEngIds;

  std::set<std::string> m_supportedConstraintTypes;
    
  std::vector<EpistemicReferenceHypothesisPtr> getHypothesisList(InferredResultPtr infRes);
  
  std::string getBeliefId(std::string atom);
  
  bool makeHypothesis(std::string id, double prob, EpistemicReferenceHypothesisPtr &h);
  
  void newConstraints(const cdl::WorkingMemoryChange & _wmc);
  void removeConstraints(const cdl::WorkingMemoryChange & _wmc);
  
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
  virtual ~MLNRefResolutionClient() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

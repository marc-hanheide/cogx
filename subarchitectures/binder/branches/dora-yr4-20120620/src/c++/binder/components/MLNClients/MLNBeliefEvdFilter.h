/**
 * @author Alen Vrecko
 * @date September 2011
 *
 * A component that filters beliefs and provides evidence to a Markov logic network engine.
 */

#ifndef MLN_BELIEF_EVD_FILTER_H
#define MLN_BELIEF_EVD_FILTER_H

#include <AbsMLNEvdFilter.h>


//using namespace de::dfki::lt::tr::beliefs::slice;

namespace cast
{

class MLNBeliefEvdFilter : public AbsMLNEvdFilter
{
 private:
   map<string,MLNFact> m_filtFacts;
   map<string,MLNFact> m_oldFacts;
//  std::vector<std::string> m_evdEngIds;
  

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
  virtual ~MLNBeliefEvdFilter() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

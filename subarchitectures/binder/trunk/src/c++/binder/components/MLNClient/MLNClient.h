/**
 * @author Alen Vrecko
 * @date May 2011
 *
 * Markov logic network engine tester.
 */

#ifndef MLN_CLIENT_H
#define MLN_CLIENT_H

//#include <vector>
//#include <string>
//#include <queue>
//#include <map>
//#include <algorithm>

//#include <boost/interprocess/sync/named_semaphore.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>

//#include <../../MLNUtils.h>
#include <MLNListener.h>
#include <MLNEvdProvider.h>
//#include <cast/architecture/ManagedComponent.hpp>

//#include <beliefs_cast.hpp>
//#include <binder.hpp>


//using namespace de::dfki::lt::tr::beliefs::slice;

namespace cast
{

class MLNClient : public MLNEvdProvider
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
  virtual ~MLNClient() {}
};

}

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

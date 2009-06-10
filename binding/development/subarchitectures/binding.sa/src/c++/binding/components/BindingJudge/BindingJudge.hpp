#ifndef BINDING_BINDING_JUDGE_H_
#define BINDING_BINDING_JUDGE_H_

#include "binding/abstr/AbstractBinder.hpp"
#include "binding/utils/BindingScoreUtils.hpp"
#include "binding/utils/non_repeating_queue.hpp"
#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <deque>
#include <set>

namespace Binding {

/// Summarizes the binding scores and selects the best union for each proxy
class BindingJudge: public AbstractBinder {
public:
  BindingJudge(const std::string &_id);
  virtual ~BindingJudge();

  virtual void runComponent(){};  
  
  virtual void start();
  
  /// Triggered by a updated \p FeatureSetComparison
  void scoreUpdated(const cast::cdl::WorkingMemoryChange & _wmc);
  /// removes the binding from all local data and regenerates the
  /// union highscore if necessary
  void unionDeleted(const cast::cdl::WorkingMemoryChange & _wmc);
  /// removes the proxy from all local data and regenerates the
  /// union highscore if necessary
  void proxyDeleted(const cast::cdl::WorkingMemoryChange & _wmc);
  
  /// together with \p unionDeleted, it makes sure \p m_prox2uni is
  /// updated
  void unionUpdated(const cast::cdl::WorkingMemoryChange & _wmc);

  /// processes the queue, if there is one...
  void bindingTaskDeleted(const cast::cdl::WorkingMemoryChange & _wmc);

protected:

  virtual void taskAdopted(const std::string &_taskID){};
  virtual void taskRejected(const std::string &_taskID){};
  virtual void configure(std::map<std::string,std::string>& _config){AbstractBinder::configure(_config);}
  
  
  struct ProxyScores {
    ProxyScores() :
      m_bestBindingScore(defaultThresholdScore()) {}
    /// maps from unionID to the score of that union, if recorded
    std::map<std::string,BindingData::BindingScore> m_unionScore;
    /// a list of the best unions of the proxy, should be empty
    /// if all union scores are not yet collected
    BindingData::BindingScore m_bestBindingScore;
    /// the list of the best_unions
    std::vector<std::string> m_bestUnions;
    /// the list of the unions that do not match
    std::vector<std::string> m_nonMatchingUnions;
  };
  
  /// maps from a proxyID to a map which maps from the unionID
  /// to the score of the comparison between the two
    //map<std::string, map<std::string, BindingData::BindingScore> > m_proxy2Scores;

  /// maps from a proxy to the best unions of the proxy
    //map<std::string, set<std::string> > m_proxy2bestUnions;

  /// the resulting scores for a proxy
  cast::StringMap<ProxyScores>::map m_proxyScores;
  /// maps from proxyID to already stored
  /// BestUnionsForProxy if it exists
  //map<std::string, std::string> m_proxy2bestUnionsID;
  
  /// a set of all binding IDs of already deleted proxies, used to
  /// ignore stray scoring results of already deleted proxies (they
  /// may come in a bit too late)
  std::set<std::string> m_deletedProxies;
  /// same as \p m_deletedProxies, but for the deleted unions
  std::set<std::string> m_deletedUnions;
  
  std::vector<std::string> _bestUnions(const std::map<std::string,BindingData::BindingScore>& _unionscores, ///< the calculated scores
				       BindingData::BindingScore& _best_score, 
				       const BindingData::BindingScore& _threshold_score,
				       const std::string& _already_bound_to,
				       const std::string& _proxyID);

  std::vector<std::string> _nonMatchingUnions(const std::string& _proxyID, 
					      const std::map<std::string,BindingData::BindingScore>::map& _bindingScores);

  void _calculateBestUnionsAndStoreIfNecessary(const std::string & _proxyID);
  
  
  /// a queue of proxies that may be appropriate for a binding task
  non_repeating_queue<std::string> m_proxyIDQueue;
  /// true iff WM has a binding task
  bool m_bindingTask;

};

} // namespace Binding

#endif // BINDING_BINDING_JUDGE_MOTIVATOR_H_

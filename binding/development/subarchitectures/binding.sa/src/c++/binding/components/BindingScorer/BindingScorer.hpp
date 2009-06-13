#ifndef BINDING_BINDING_SCORER_H_
#define BINDING_BINDING_SCORER_H_

#include "binding/abstr/AbstractBinder.hpp"
#include "ComparatorCache.hpp"
#include "binding/utils/BindingScoreUtils.hpp"
#include "binding/idl/BindingQueries.hh"
#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>

namespace Binding {

/// thrown when the comparison betwee two features needs to be made externally
/// \remark the \p featureComparisonID is always locked completely, so it must be unlocked when the exception is caught
struct ExternalScoreNotReadyException{
  ExternalScoreNotReadyException(const std::string& _featureComparisonID) : featureComparisonID(_featureComparisonID) {}
  /// the id f the comparison result that should be overwritten on WM.
  const std::string featureComparisonID;
};

/// Calculates scores between proxies and unions
class BindingScorer: public AbstractBinder {
  friend class ComparatorCache;
public:
  BindingScorer(const std::string &_id);
  virtual ~BindingScorer();
  
  virtual void runComponent();
  
  virtual void start();
  
  void scoreThisUnion(const cast::cdl::WorkingMemoryChange & _wmc);

  void bindTheseProxiesAdded(const cast::cdl::WorkingMemoryChange & _wmc);
    
  void scoringTaskAdded(const cast::cdl::WorkingMemoryChange & _wmc);
  
  /// Triggered by an update FeatureComparison
  void updateComparison(const cast::cdl::WorkingMemoryChange & _wmc);
  
  /// called when a comparator registers that it's ready to compare things
  void registerComparatorCompetence(const cast::cdl::WorkingMemoryChange & _wmc);

  /// responds to \p BindingQueries::BasicQuery
  void answerBasicQuery(const cast::cdl::WorkingMemoryChange & _wmc);
  /// responds to \p BindingQueries::AdvancedQuery
  void answerAdvancedQuery(const cast::cdl::WorkingMemoryChange & _wmc);

  /// a proxy is updated, it could be that the binder just bound it
  void proxyUpdateReceived(const cast::cdl::WorkingMemoryChange & _wmc);

  /// ...
  void binderStatusReceived(const cast::cdl::WorkingMemoryChange & _wmc);
  /// removes a proxyID from \p processedProxies
  void proxyProcessingFinished(const cast::cdl::WorkingMemoryChange & _wmc);

  
protected:

  virtual void taskAdopted(const std::string &_taskID){};
  virtual void taskRejected(const std::string &_taskID){};
  virtual void configure(std::map<std::string,std::string>& _config);


  /// scores a proxy vs a union and stores the result
  void scoreAndStore(const LBindingProxy& _proxy,
		     const LBindingUnion& _union);
  /// scores a proxy vs NIL, i.e. before it is bound. And also stores the result.
  void scoreAndStoreVsNIL(const LBindingProxy& _proxy);
  /// scores a proxy vs a union
  BindingData::BindingScore scoreProxyVsUnion(const LBindingProxy& _proxy,
					      const LBindingUnion& _union);
  
private:

  /// scores a proxy vs a union w.r.t. the relational structures
  void _relational_score(const LBindingProxy& _proxy,
			 const LBindingUnion& _union,
			 unsigned int& _score,
			 bool& _mismatch);
  
  /// stores the score...
  void _storeScore(const std::string& _proxyID,
		   const std::string& _unionID,
		   const BindingData::BindingScore& _score);
    
  
private:



  /// test... states that this is the nth process of the same
  /// kind... used to select if it should be triggered by a task or
  /// not
  unsigned int nth;
  /// total number of processes
  unsigned int maxN;

  ComparatorCache comparatorCache;

  //const BindingFeatureOntology::TrustSpecMap& internalTrust() const {return BindingFeatureOntology::internalTrust();}
  

//  CASTDataCache<BindingData::FeatureComparison> featureComparisonCache;

  /// used to guess which of several unions seems to be most salient
  double _salienceHeuristics(const std::vector<BindingFeatures::Salience>& proxy_saliences,
			     const std::vector<BindingFeatures::Salience>& union_saliences,
			     const std::vector<BindingFeatures::CreationTime>& proxy_creation_times,
			     const std::vector<BindingFeatures::CreationTime>& union_creation_times) const;

  /// if a signalled comparisonID is in this list, it means that it an
  /// open BindingQueries::AdvancedQuery should be answered by it
  /// \remark maps into the queryID
  std::map<std::string,std::string> openAdvancedQueryFeatureComparisonIDs;
  /// if a signalled comparisonID is in this list, it means that it an
  /// open BindingQueries::BasicQuery should be answered by it
  /// \remark maps into the queryID
  std::map<std::string,std::string> openBasicQueryFeatureComparisonIDs;
  
  /// used internally
  void _answerBasicQuery(const BindingQueries::BasicQuery& _query,
			 const std::string& _queryID);
  void _answerAdvancedQuery(const BindingQueries::AdvancedQuery& _query,
			    const std::string& _queryID);

private:
  /// a list of currently processed proxies (that were received in a
  /// \p BindingData::BindTheseProxies struct)
  std::set<std::string> processedProxies;


  /// acquires a token that will be owned by either the binding SA or a monitor
  virtual void acquireBinderToken() 
  {
    acquireToken(_binderTokenAddress());
//    if(hasBinderTokenToken())
//      releaseBinderTokenToken();
  }
  /// releases the token that will be owned by either the binding SA or a monitor
  virtual void releaseBinderToken() 
  {
    releaseToken(_binderTokenAddress());
//    acquireBinderTokenToken();
  }
  
  /// if true, then the scorer only runs until tokens are created,
  /// then locks forever.
  bool dummy;

};



} // namespace Binding

#endif // BINDING_BINDING_SCORER_H_

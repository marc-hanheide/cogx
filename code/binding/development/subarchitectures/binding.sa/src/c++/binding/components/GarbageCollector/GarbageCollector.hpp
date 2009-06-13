#ifndef BINDING_BINDING_DOT_VIEWER_H_
#define BINDING_BINDING_DOT_VIEWER_H_

#include <boost/shared_ptr.hpp>
#include <map>
#include <set>
#include <balt/core/StringMap.hpp>
#include "binding/abstr/AbstractBinder.hpp"
#include "binding/utils/DotUtils.hpp"
#include "binding/utils/LocalClasses.hpp"


namespace Binding {

/// Cleans up the WM, and performs some "police" tasks too
class GarbageCollector : public AbstractBinder {
public:
  GarbageCollector(const std::string &_id);
  virtual ~GarbageCollector();
  
  /// override start method to set change filters
  virtual void start();
  virtual void configure(std::map<std::string, std::string>& _config);

  /// called when \p DeletedBindingProxy is added by the binder,
  /// meaning that it's safe to delete
  void deletedProxyAdded(const cast::cdl::WorkingMemoryChange&);
  /// deletes scores associated with the deleted union
  void unionDeleted(const cast::cdl::WorkingMemoryChange&);
  /// stores the binding score address so that it can be deleted when
  /// the associated \it union is deleted (not proxy, since we want to
  /// avoid deleting it twice)
  void bindingScoreAdded(const cast::cdl::WorkingMemoryChange&);
  /// stores the feature comparison address so that it can be deleted
  /// when the associated proxy is deleted
  void featureComparisonAdded(const cast::cdl::WorkingMemoryChange&);
  
  /// for when a \p BindingData::FeaturePointerDeletionTask is added
  /// (which happens if a proxy is updated with new features and old
  /// ones are discarded).
  void explicitFeatureDeletionAdded(const cast::cdl::WorkingMemoryChange&);
  
  /// called if illegal signals are detected, so far only feature overwrites
  void reportIllegalSignal(const cast::cdl::WorkingMemoryChange&);
  
  /// checks the status and starts deletion if stable and aborts as
  /// soon as noo longer stable, the statusCache is used to keep
  /// track of the status
  void statusUpdated(const cast::cdl::WorkingMemoryChange&);

protected:
  virtual void taskAdopted(const std::string &_taskID) {};
  virtual void taskRejected(const std::string &_taskID) {};
  virtual void runComponent();
private:
  /// checks that config has been called
  bool confCalled;
  enum PoliceMode {
    STRICT, ///< aborts if something is fishy (default)
    WARNING, ///< prints warnings on stderr
    DISABLED ///< not used at all, cheaper but not good during testing
  };
  PoliceMode policeMode;
  /// contains IDs associated to a proxy or union
  struct AssociatedIDs {
    std::set<std::string> ids;
  };
  cast::StringMap<AssociatedIDs>::map associatedToProxy;
  cast::StringMap<AssociatedIDs>::map associatedToUnion;
  std::deque<std::string> deletionQueue;
  void deleteOneOnQueue();

  /// caches the status of the binder
  std::auto_ptr<cast::CachedCASTData<BindingData::BinderStatus> > statusCache;

};

} // namespace Binding

#endif // BINDING_BINDING_DOT_VIEWER_H_

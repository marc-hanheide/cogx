#ifndef BINDING_GROUP_MANAGER_H_
#define BINDING_GROUP_MANAGER_H_

#include "binding/abstr/AbstractBindingWMRepresenter.hpp"
#include "binding/abstr/AbstractMonitor.hpp"
#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>

namespace Binding {

/// Spawns of individual proxies from group proxies
class BindingGroupManager : 
    public AbstractMonitor,
    public AbstractBindingWMRepresenter {
public:
  BindingGroupManager(const std::string &_id);
  virtual ~BindingGroupManager();

  virtual void runComponent(){};  
  
  virtual void start();
  
  void proxyAdded(const cast::cdl::WorkingMemoryChange& _wmc);
  void proxyUpdated(const cast::cdl::WorkingMemoryChange& _wmc);
  void proxyDeleted(const cast::cdl::WorkingMemoryChange& _wmc);

  void unionAdded(const cast::cdl::WorkingMemoryChange& _wmc);
  void unionUpdated(const cast::cdl::WorkingMemoryChange& _wmc);
  void unionDeleted(const cast::cdl::WorkingMemoryChange& _wmc);
  
  BindingData::FeaturePointers 
  _copy_feature_pointers(const LBindingProxy& _proxy, 
			 const std::set<std::string>& _exclude);
protected:

  virtual void taskAdopted(const std::string &_taskID){};
  virtual void taskRejected(const std::string &_taskID){};
  virtual void configure(std::map<std::string,std::string>& _config);
  
  struct GroupInfo {
    GroupInfo(unsigned int _size) : size(_size), current_size(0) {}
    unsigned int size;
    unsigned int current_size;
    std::set<std::string> group_member_ids;
  };
  
  /// all groups with som info
  std::map<std::string,GroupInfo> group_info;
  /// all singles and their groups
  std::map<std::string, std::string> single2group;
  /// the IDs of all generated singularss that are only bound to themselves
  std::set<std::string> unbound_singles;
  /// maps between groupIDs that have members that have been bound,
  /// which is only allowed once (otherwise unbound groups would grow
  /// infinitely). Maps both ways.
  std::set<std::pair<std::string,std::string> > groups_bound_through_members;
  
  /// stores uni2prox only of singulars
  std::map<std::string,std::set<std::string> > uni2prox; 
  
  const BindingFeatures::Group&
  _retrieve_group_info(const LBindingProxy&);

  std::string _create_and_store_singular(const std::string& groupID, const LBindingProxy& group);
  
  /// makes a copy of the Relation Proxy at RelationID, and replaces
  /// the references to the groupID to the SinularID
  std::string _copy_relation_proxy(const std::string& relationID, 
				   const std::string& singularID);

  
  /// checks if a union update affected any of the members of a group
  /// and adds a new singular of that group if necessary
  void _check_union(const std::string& _unionID);

  
};

} // namespace Binding

#endif // BINDING_GROUP_MANAGER_H_

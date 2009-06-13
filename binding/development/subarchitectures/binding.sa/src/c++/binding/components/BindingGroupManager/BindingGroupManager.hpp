#ifndef BINDING_GROUP_MANAGER_H_
#define BINDING_GROUP_MANAGER_H_

//#include "binding/abstr/AbstractBindingWMRepresenter.hpp"
#include "binding/abstr/AbstractMonitor.hpp"
#include <boost/logic/tribool.hpp>
#include <boost/shared_ptr.hpp>
#include <map>
#include <set>

namespace Binding {

/// Spawns of individual proxies from group proxies
class BindingGroupManager : 
    public AbstractMonitor
//    public AbstractBindingWMRepresenter 
{
public:
  BindingGroupManager(const std::string &_id);
  virtual ~BindingGroupManager();

  virtual void runComponent(){};  
  
  virtual void start();
  
  /// when groupdetails are added, then there will soon be a
  /// groupproxy, so the ID of this proxy can now be added to the list
  void groupDetailsAdded(const cast::cdl::WorkingMemoryChange& _wmc);
  //void proxyAdded(const cast::cdl::WorkingMemoryChange& _wmc);
  /// a proxy is perhaps bound, now check if it is a grou or a singular
  void proxyUpdated(const cast::cdl::WorkingMemoryChange& _wmc);
  void proxyDeleted(const cast::cdl::WorkingMemoryChange& _wmc);
  
protected:
  BindingData::FeaturePointers 
  _copy_feature_pointers(const LBindingProxy& _proxy, 
			 const std::set<std::string>& _exclude);

  virtual void taskAdopted(const std::string &_taskID){};
  virtual void taskRejected(const std::string &_taskID){};
  virtual void configure(std::map<std::string,std::string>& _config);
  
  std::set<std::string> groupProxyIDs;
  /// maps from the group's ID to the individuals' IDs
//  cast::StringMap<std::set<std::string> >::map individualProxyIDs;
    std::set<std::string> individualProxyIDs;

  std::string _create_and_store_singular(const LBindingProxy& _group_original);
  
//  /// makes a copy of the Relation Proxy at RelationID, and replaces
//  /// the references to the groupID to the SingularID
//  std::string _copy_relation_proxy(const std::string& relationID, 
//				   const std::string& singularID);

  
  /// checks if a union update affected any of the members of a group
  /// and adds a new singular of that group if necessary
  void _check_union(const std::string& _unionID);

  
};

} // namespace Binding

#endif // BINDING_GROUP_MANAGER_H_

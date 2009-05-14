#ifndef BINDING_MOTIVE_GENERATOR_H_
#define BINDING_MOTIVE_GENERATOR_H_


#include "BindingMotiveTemplates.hpp"

#include <binding/idl/BindingData.hh>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <binding/utils/GraphLoader.hpp>
#include <planning/util/TemporaryPlanningState.hpp>
#include <cast/architecture/ManagedProcess.hpp>
#include <cast/core/CASTDataCache.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <motivation/idl/MotivationData.hh>
#include <ext/hash_set>

/**
 * The core functionality for the for the motive generator. This
 * component is intended to monitor a binding wm and match the proxies
 * there against motive templates. The templates are effectively
 * looking for particular subgraphs of binding structures. Once a template has been fully matched (insta

 * This class should be inherited from for a particular set of motives
 * that are registered with registerMotiveTemplate.
 */
class BindingMotiveGenerator : 
  public cast::ManagedProcess,
  public Binding::AbstractBindingWMRepresenter  {

public:

  BindingMotiveGenerator(const std::string &_id);
    
  /// overridden start method used to set filters
  virtual void start();
  virtual void configure(std::map<std::string,std::string> & _config);
  

protected:
  virtual void taskAdopted(const std::string &_taskID){}
  virtual void taskRejected(const std::string &_taskID){}
  virtual void runComponent(){}


  /**
   * Provide details of the agent for the planning process/
   */
  virtual std::string domainName() const = 0;
  /**
   * Provide details of the agent for the planning process/
   */
  virtual std::string domainFile() const = 0;

  /**
   * Called when any proxy is deleted on binding wm.
   */
  void triggerTypeAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  /**
   * Called when the variable list is added
   */
  void avmListAdded(const cast::cdl::WorkingMemoryChange& _wmc);

  /**
   * Generate new motive from template.
   */
  void generateMotive(AbstractMotiveTemplate * _mt);



  /**
   * Used to register motive templates. MotiveT should be a subclass
   * of AbstractMotiveTemplate.
   */
  template <class TriggerT, class MotiveT>
  void registerMotiveTemplate() {
    m_motiveTemplates.push_back(new MotiveT(*this, m_motiveHandler));
    
    const string & type(cast::typeName<TriggerT>());

    //if this is a new type to trigger on
    if(m_triggerTypes.find(type) == m_triggerTypes.end()) {
      //create a filter to listen
      addChangeFilter(cast::createLocalTypeFilter<TriggerT>(cast::cdl::ADD),
		      new cast::MemberFunctionChangeReceiver<BindingMotiveGenerator>(this, 
										     &BindingMotiveGenerator::triggerTypeAdded));

  

    }

  }

  //std::string m_mainBindingSA;
  
  /// will do sth with an ambiguity. we'll see what... 
  void makeSthWithAnAmbiguity(const BindingData::Ambiguity& _ambiguity);

private:

  class MotiveResultsReceiver : public cast::WorkingMemoryChangeReceiver {
  public:
    MotiveResultsReceiver(BindingMotiveGenerator & _component, 
			  AbstractMotiveTemplate * _mt,
			  const std::string & _motiveID) 
      : m_component(_component),
	m_template(_mt),
	m_motiveID(_motiveID) {}
    void workingMemoryChanged(const cast::cdl::WorkingMemoryChange& _wmc);    
  private:
    BindingMotiveGenerator & m_component;
    AbstractMotiveTemplate* m_template;
    std::string m_motiveID;
  };

  friend class MotiveResultsReceiver;

  /**
   * Checks whether all source proxies with SourceData features are
   * unambiguously bound
   */
  bool hasAmbiguousSource(AbstractMotiveTemplate *_mt);

  typedef cast::StringMap<std::string>::map StringStringMap;

  void loadAVMs(const std::string _avmID, StringStringMap & _map);

  ///utility object for loading data from motive sa
  Binding::BindingGraphHandler m_motiveHandler;

  ///list of motive templates
  std::vector<AbstractMotiveTemplate *> m_motiveTemplates;

  ///Cache for variable mappings. As there will probably be only one
  ///mapping list this may be overkill.
  cast::CASTDataCache<motivation::idl::AddressVariableMappings> m_avmCache;

  ///Receiver to get teh address of the avm list
  cast::WorkingMemoryChangeReceiver * m_avmReceiver;

  ///location of avm on wm
  std::string m_avmID;

  typedef __gnu_cxx::hash_set<std::string> StringSet;
  //set of types being used to trigger motives
  StringSet m_triggerTypes;

};


#endif //BINDING_MOTIVE_GENERATOR_H_

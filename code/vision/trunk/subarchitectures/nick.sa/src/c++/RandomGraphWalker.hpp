#ifndef RANDOM_GRAPH_WALKER_HPP_
#define RANDOM_GRAPH_WALKER_HPP_

#include <cast/architecture/ManagedComponent.hpp>
#include <autogen/NavData.hpp>
#include <autogen/PTZ.hpp>

//fwd decl
class CommandListener;

/**
 * An abstract class to represent a component in a subarchitecture that
 * can read from a working memory and does have its operations
 * controlled by a task manager.
 * 
 * @author nah
 */
class RandomGraphWalker : 
  public cast::ManagedComponent {
  
public:
  
  RandomGraphWalker();
  virtual ~RandomGraphWalker(){};

protected:
  virtual
  void
  runComponent();

  virtual 
  void 
  start();
  
  virtual void 
  configure(const std::map<std::string,std::string> & _config);

private:

  friend class CommandListener;

  void 
  graphAdded(const cast::cdl::WorkingMemoryChange & _wmc);

  void 
  nodeAdded(const cast::cdl::WorkingMemoryChange & _wmc);

  bool
  commandOverwritten(const cast::cdl::WorkingMemoryChange & _wmc);

  void 
  visitNextNode();

  void 
  say(const std::string & _message);

  void
  connectToPTZServer();

  std::vector<NavData::FNodePtr> m_nodesToVisit;
  std::vector<NavData::FNodePtr> m_storedNodes;

  std::string m_navSA;

  bool m_loop;
  
  ///the machine to connect to for ptz action
  std::string m_ptzHost;

  ///proxy for the ptz server
  ptz::PTZInterfacePrx m_ptzServer;

};


class CommandListener: public cast::WorkingMemoryChangeReceiver  {
public:
  CommandListener(RandomGraphWalker & _component) : m_component(_component) {}

  void workingMemoryChanged(const cast::cdl::WorkingMemoryChange & _wmc) {
    if(m_component.commandOverwritten(_wmc)) {
      //remove this filter
      m_component.removeChangeFilter(this, cast::cdl::DELETERECEIVER);	    
    }
  }

private:
  RandomGraphWalker & m_component;  
};

#endif

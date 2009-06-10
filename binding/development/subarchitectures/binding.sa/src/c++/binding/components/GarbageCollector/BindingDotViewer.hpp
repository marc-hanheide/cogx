#ifndef BINDING_BINDING_DOT_VIEWER_H_
#define BINDING_BINDING_DOT_VIEWER_H_

#include <boost/shared_ptr.hpp>
#include <map>
#include <set>
#include "binding/abstr/AbstractBinder.hpp"
#include "binding/utils/DotUtils.hpp"
#include "binding/utils/LocalClasses.hpp"


namespace Binding {

/// Writes dot-files based on all the unions and proxies on WM
class BindingDotViewer : public AbstractBinder {
public:
  BindingDotViewer(const std::string &_id);
  virtual ~BindingDotViewer();

  
  /// override start method to set change filters
  virtual void start();

  /// generates and store a dot graph based on current WM contents
  void generateDotGraph(const cast::cdl::WorkingMemoryChange&);  
  
  /// calls generateDotGraph only if the status of the binder is that it is stable
  void generateDotGraphIfStable(const cast::cdl::WorkingMemoryChange&);  

  
public:
  struct DotParameters {
    enum ShowMode {show_full,only_source,only_dots,show_short,no_show};
    DotParameters() :
      unions(show_full),
      proxies(show_full) {}
    ShowMode unions;
    ShowMode proxies;
  };
  
  /// reads from WM and translates into a dot graph
  std::string dotGraph();
  
protected:
  virtual void taskAdopted(const std::string &_taskID) {};
  virtual void taskRejected(const std::string &_taskID) {};
  virtual void configure(std::map<std::string,std::string>& _config);
  virtual void runComponent();
private:
    
  /// prints all dot relations
  //std::string _printDotRelations() const;

  /// where dot files will be saved
  std::string m_dotPath;
  /// counts number of dot-files
  unsigned int m_dotCount;
  /// DotParameters used to configure how the dot graph should look
  
  DotParameters m_dotParameters;

  /// prints the relations
  std::string _dotRelations();

  std::string
  _retrieve_singular_groupID(const LBindingProxy&);
  std::set<std::string>
  _retrieve_singular_groupIDs(const LBindingUnion&);

  
  /*struct testFct {
    std::string operator()(const boost::shared_ptr<const BindingData::BindingProxy>& _proxy) const {return std::string(_proxy->m_unionID);}
  };*/
    
  /// the version of status whan last plotting
  int m_lastStatusVersion;
  /// Maps from proxy to the binding of that proxy (CAST IDs)
  std::map<std::string,std::string> m_prox2uni;
  
};

} // namespace Binding

#endif // BINDING_BINDING_DOT_VIEWER_H_

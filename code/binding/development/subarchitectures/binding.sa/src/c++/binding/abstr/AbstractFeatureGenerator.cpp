#include "AbstractFeatureGenerator.hpp"


namespace Binding {

  using namespace std;
  using namespace cast;
  using namespace boost;

  ///do the registration
//   void 
//   AbstractFeatureGenerator::registerFeatureGenerator(const std::string & _type,
// 						     cast::WorkingMemoryChangeReceiver * _pReceiver) {

//   }

  AbstractFeatureGenerator::AbstractFeatureGenerator(const std::string & _id) 
    :WorkingMemoryAttachedComponent(_id),       
     AbstractMonitor(_id),      
      m_filterUp(false) { 
  }



  void 
  AbstractFeatureGenerator::handleGenerationCommand(const cast::cdl::WorkingMemoryChange & _wmc) {
    log("AbstractFeatureGenerator::handleGenerationCommand");
  }

} //namespace Binding

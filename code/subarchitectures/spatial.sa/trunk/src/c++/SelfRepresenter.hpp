//
// = Filename
//   SelfRepresenter.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef SelfRepresenter_hpp
#define SelfRepresenter_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <SpatialProperties.hpp>
//#include <BinderEssentials.hpp>
//#include <Marshalling.hpp>
#include <NavData.hpp>
#include <string>
#include <map>

using namespace std;
using namespace cast;

namespace spatial {
class SelfRepresenter: public cast::ManagedComponent {

public:
  /**
   * Constructor
   */
  SelfRepresenter();

  /**
   * Destructor
   */
  virtual ~SelfRepresenter();

  virtual void start();
  virtual void stop();
  virtual void runComponent();
  virtual void configure(const std::map<std::string, std::string>& _config);

private:
  NavData::FNodePtr getCurrentNavNode();
};
}
;

#endif


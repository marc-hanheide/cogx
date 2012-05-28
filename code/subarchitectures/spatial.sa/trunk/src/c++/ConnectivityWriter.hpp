//
// = Filename
//   ConnectivityWriter.hpp
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

#ifndef ConnectivityWriter_hpp
#define ConnectivityWriter_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <SpatialProperties.hpp>
#include <BinderEssentials.hpp>
#include <Marshalling.hpp>
#include <string>
#include <map>

using namespace std;
using namespace cast;
using namespace binder::autogen::core;

namespace spatial {
class ConnectivityWriter: public cast::ManagedComponent {

public:
  /**
   * Constructor
   */
  ConnectivityWriter();

  /**
   * Destructor
   */
  virtual ~ConnectivityWriter();

  virtual void start();
  virtual void stop();
  virtual void runComponent();
  virtual void configure(const std::map<std::string, std::string>& _config);

private:
  void newConnectivity(const cdl::WorkingMemoryChange &wmc);
  void changedGateway(const cdl::WorkingMemoryChange &wmc);

  Marshalling::MarshallerPrx m_marshaller;
};
}
;

#endif


#include "SonarPullClient.hpp"

using namespace std;
using namespace Ice;
using namespace cast;
using namespace cast::cdl;
using namespace Sonar;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new SonarPullClient();
  }
}

SonarPullClient::SonarPullClient() : m_sonarServerName("sonar.server") {

}

void 
SonarPullClient::configure(const std::map<std::string,std::string> & config) {

  std::map<std::string,std::string>::const_iterator it;
  if ((it = config.find("--sonar-server")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_sonarServerName;
  }
  log("Using m_sonarServerName=%s", m_sonarServerName.c_str());

}


void
SonarPullClient::runComponent() {
  SonarServerPrx server(getIceServer<SonarServer>(m_sonarServerName));
  
  //we'll probably get an exeption before this if fails
  SonarScan2d scan;
  if(server) {
    while(isRunning()) {
      scan = server->pullSonarScan2d();
      println("scan: %d", scan.ranges.size());
      for(RangeSequence::const_iterator i = scan.ranges.begin();
	  i < scan.ranges.end(); ++i) {
	println("reading: %f", *i);
      }
    }
  }

}

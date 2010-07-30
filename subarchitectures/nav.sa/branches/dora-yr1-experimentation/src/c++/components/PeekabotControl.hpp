//
// = FILENAME
//   PeekabotControl.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef PeekabotControl_hpp
#define PeekabotControl_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <NavData.hpp>
#include <SpatialData.hpp>
#include <peekabot.hh>
#include <peekabot/Types.hh>

namespace navsa {

/**
 * This class allows you to control the robot using peekabot, by
 * dragging an icon on the screen to where you want the robot to go.
 *
 * @param -c a CURE config file to optionally read the
 * PEEKABOT_HOST and PEEKABOT_ROBOTXMLFILE. The first are to say where
 * the peekabot server is running and the second gives the name of the
 * xml file with the robot definition.
 * @param --retry-interval  How many secs to wait to try to connect again to the peekabot server (default 10s). Negative value mean no retries
 * @param --action what action to perform 
 *
 * @author Patric Jensfelt
 */
class PeekabotControl : public cast::ManagedComponent
{
public:
  PeekabotControl();
  virtual ~PeekabotControl();
  
protected:

  virtual void configure(const std::map<std::string, std::string>& config);
  virtual void start();
  virtual void runComponent();

  void connectPeekabot();

  std::string m_PbHost;
  int m_PbPort;

  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry

  int m_CtrlAction;
  SpatialData::PlaceIDSeq placeseq;
  peekabot::PeekabotClient m_PeekabotClient;  
};

}; // namespace navsa

#endif // PeekabotControl_hpp
